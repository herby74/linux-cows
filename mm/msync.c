/*
 *	linux/mm/msync.c
 *
 * Copyright (C) 1994-1999  Linus Torvalds
 */

/*
 * The msync() system call.
 */
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/pagemap.h>
#include <linux/rmap.h>
#include <asm/tlb.h>
#include <asm/pgtable.h>
#include <linux/mmdebug.h>


/* on x86 we have  
	HAVE_GENERIC_MMU_GATHER
	! CONFIG_HAVE_RCU_TABLE_FREE
	CONFIG_MMU
	CONFIG_DEBUG_BUGVERBOSE	
*/

struct handle_private {
        struct mmu_gather *tlb;
        struct address_space *mapping;
	int anon_removed;
	unsigned long end;
};

static int handle_entry( pte_t *ptep , unsigned long addr, unsigned long next, struct mm_walk *walk  ) {
	struct handle_private *private;
	struct mmu_gather *tlb;
	struct page *vp, *fp;
	struct anon_vma *anon; 
	pgoff_t offset;
	struct vm_area_struct *vma;
	int error = 0 ;
	pte_t pte;
	
	private = ( struct handle_private * ) walk -> private;
	tlb = private -> tlb;
	pte = *ptep; // todo sync  ptep_get_and_clear( NULL, 0, ptep ); see source/arch/x86/include/asm/pgtable.hi 
	// http://lxr.free-electrons.com/source/arch/x86/include/asm/pgtable_64.h#L47
	if ( pte_none( pte ) ){
		return 0; 
	}
	vp = pte_page( pte );
	anon = page_anon_vma( vp );
	VM_BUG_ON_PAGE(  1 !=  page_count( vp), vp ); 
	// spin_unlock( & walk->mm->page_table_lock );
	// vpte = ptep_get_and_clear( NULL, 0,  ptep); >>  later 
	// get_page( vp );	
	// printk( KERN_DEBUG "checking page flags=%lx map=%p index=%lu pte=%lx \n", vp-> flags,  (void *)vp->mapping, vp->index, pte_val( *ptep )  ); 
	// VM_BUG_ON( vp -> mapping == NULL );
	if ( anon  ){
		// VM_BUG_ON_PAGE( PageAnon( vp ), vp);
		// ??? do we need a lock on page_table ? spin_lock( & walk->mm->page_table_lock );
		// ??? do we need to lock the page ? 
		if ( private -> mapping ){
			vma = walk -> vma;
			offset = vma -> vm_pgoff + ( ( addr - vma->vm_start) >> PAGE_SHIFT  );
			fp = find_get_page( private -> mapping, offset );       
			if ( unlikely( ! fp ) ){
				error = -ENOMEM;
				printk( KERN_ERR  " No file page in cache " );
				goto no_copy;
			}
	                VM_BUG_ON( page_to_pgoff( vp ) != page_to_pgoff( fp ) );
			copy_highpage( fp ,vp );
			SetPageDirty( fp );
			put_page( fp);
		}
		// see include/asm-x86_64/pgtable.h 
		// ?? lock_page(vp);
		SetPageDirty( vp ); 
		page_remove_rmap( vp , 0  ); // decrements _mapcount 
		private -> anon_removed++;
  		// dec_mm_counter_fast(  vma -> vm_mm , MM_ANONPAGES ); => done later with anon_removed
		pte = ptep_get_and_clear( NULL, 0, ptep );
		VM_BUG_ON_PAGE( pte_val( pte ) !=  pte_val( *ptep) , vp );
		// tlb_remove_page( tlb, vp );
		if (__tlb_remove_page(tlb, vp )) {
	                tlb_flush_mmu(tlb);
			tlb -> start = addr;
			tlb -> end = private -> end; // reset end as tlb_flush_mmu calls __tlb_reset_range which resets the range.
		}
		// single __flush_tlb_one(addr); is probably faster for a few pages 
		// printk( KERN_DEBUG "write_or_copy=%p page=%p addr=%0lx pcount=%d prefcount=%d", private -> mapping , vp , addr, page_count( vp), page_mapcount( vp)  );
		// put_page( vp );
		// error = free_pages_check_bad( vp );
		// VM_BUG_ON_PAGE( page_count( vp )  , vp ); 
	}  
no_copy:
	return error;
}

/* not used 
static int priv_shm_test_walk (unsigned long addr, unsigned long next, struct mm_walk *walk){
	return  walk -> vma -> vm_flags & VM_SHARED; // include if not shared 
}*/

// see  mmap.c:unmap_region, and memory.c:zap_page_range_single
static int flush_private(struct vm_area_struct *vma, unsigned long start, unsigned long end , int copy ){
	int error;
	struct mm_struct *mm = vma -> vm_mm; 
	struct mmu_gather tlb; 
	struct handle_private private = {
		.tlb = &tlb,
		.mapping = ( copy ? vma -> vm_file -> f_mapping : NULL),
		.anon_removed = 0,
		.end = end
	};
	struct mm_walk walk = {
		.pte_entry = &handle_entry,
		.pmd_entry = NULL,
		.pte_hole = NULL,
		.hugetlb_entry = NULL,
		.test_walk = NULL, // &priv_shm_test_walk,
		.mm = mm,
		.private = &private
	}; 
	
	lru_add_drain();
	tlb_gather_mmu( &tlb, mm, start, end );
	tlb.start = start; 
	tlb.end = end; 
	update_hiwater_rss(mm);
	error =	walk_page_range( start, end, &walk );
        tlb_finish_mmu(&tlb, start, end);
	if ( private.anon_removed ){
		add_mm_counter( vma -> vm_mm, MM_ANONPAGES, -private.anon_removed );
	}
	return error; 
}

/*
 * MS_SYNC syncs the entire file - including mappings.
 *
 * MS_ASYNC does not start I/O (it used to, up to 2.5.67).
 * Nor does it marks the relevant pages dirty (it used to up to 2.6.17).
 * Now it doesn't do anything, since dirty pages are properly tracked.
 *
 * The application may now run fsync() to
 * write out the dirty pages and wait on the writeout and check the result.
 * Or the application may run fadvise(FADV_DONTNEED) against the fd to start
 * async writeout immediately.
 * So by _not_ starting I/O in MS_ASYNC we provide complete flexibility to
 * applications.
 */
SYSCALL_DEFINE3(msync, unsigned long, start, size_t, len, int, flags)
{
	unsigned long end;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;
	int unmapped_error = 0;
	int error = -EINVAL;
	int write_or_drop ; 

	if (flags & ~(MS_ASYNC | MS_INVALIDATE | MS_SYNC | MS_PRIV_WRITE | MS_PRIV_DROP ))
		goto out;
	if (offset_in_page(start))
		goto out;
	if ((flags & MS_ASYNC) && (flags & MS_SYNC))
		goto out;
	if ((flags & MS_PRIV_WRITE) && (flags & MS_PRIV_DROP))
		goto out;
	write_or_drop = flags & ( MS_PRIV_WRITE | MS_PRIV_DROP );
	error = -ENOMEM;
	len = (len + ~PAGE_MASK) & PAGE_MASK;
	end = start + len;
	if (end < start)
		goto out;
	error = 0;
	if (end == start)
		goto out;
	/*
	 * If the interval [start,end) covers some unmapped address ranges,
	 * just ignore them, but return -ENOMEM at the end.
	 */
	down_read(&mm->mmap_sem);
	vma = find_vma(mm, start);
	for (;;) {
		struct file *file;
		loff_t fstart, fend;

		/* Still start < end. */
		error = -ENOMEM;
		if (!vma)
			goto out_unlock;
		/* Here start < vma->vm_end. */
		if (start < vma->vm_start) {
			start = vma->vm_start;
			if (start >= end)
				goto out_unlock;
			unmapped_error = -ENOMEM;
		}
		/* Here vma->vm_start <= start < vma->vm_end. */
		if ((flags & MS_INVALIDATE) &&
				(vma->vm_flags & VM_LOCKED)) {
			error = -EBUSY;
			goto out_unlock;
		}
		file = vma->vm_file;
		if ((flags & MS_SYNC) && file &&
				(vma->vm_flags & VM_SHARED)) {
			fstart = (start - vma->vm_start) + ((loff_t)vma->vm_pgoff << PAGE_SHIFT);
			fend = fstart + (min(end, vma->vm_end) - start) - 1;
			get_file(file);
			up_read(&mm->mmap_sem);
			error = vfs_fsync_range(file, fstart, fend, 1);
			fput(file);
			start = vma->vm_end;
			if (error || start >= end)
				goto out;
			down_read(&mm->mmap_sem);
			vma = find_vma(mm, start);
		} /* YYYZZZ */ else if ( (flags & MS_SYNC) && write_or_drop  
				&& file && ~(vma->vm_flags & VM_SHARED)) {
			get_file(file);
                        up_read(&mm->mmap_sem);		// flush with mmap_sem down ? No, use get_user_pages_fast  
			error = flush_private( vma, start, min(end, vma->vm_end), flags & MS_PRIV_WRITE ); 
			fput(file);
			start = vma->vm_end;
                        if (error || start >= end)
                                goto out;
                        down_read(&mm->mmap_sem);
                        vma = find_vma(mm, start);
		} else {	
			start = vma->vm_end;
			if (start >= end) {
				error = 0;
				goto out_unlock;
			}
			vma = vma->vm_next;
		}
	}
out_unlock:
	up_read(&mm->mmap_sem);
out:
	return error ? : unmapped_error;
}
