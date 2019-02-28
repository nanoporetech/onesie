/**
 * dma.c
 *
 * Copyright (C) 2019 Oxford Nanopore Technologies Ltd.
 *
 * Author: <info@nanoporetech.com>
 *
 * Hardware and memory interaction to do scatter-gather DMA
 *
 */
#include <linux/types.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>
#include <linux/pci.h>
#include <linux/dmapool.h>
#include <linux/spinlock.h>

#include "ont_minit1c.h"
#include "ont_minit_ioctl.h"
#include "ont_minit1c_reg.h"
#include "dma.h"

/**
 * @brief Dump a descriptor chain element
 * @param desc pointer to the descriptor
 */
static void dump_descriptor(minit_dma_extdesc_t* desc)
{
    u64 big_no;
    printk(KERN_ERR"Descriptor at virt %p\n",desc);
    big_no = desc->read_hi_phys;
    big_no <<= 32;
    big_no |= desc->read_lo_phys;
    printk(KERN_ERR"read physical address 0x%016llx\n", big_no);
    big_no = desc->write_hi_phys;
    big_no <<= 32;
    big_no |= desc->write_lo_phys;
    printk(KERN_ERR"write physical address 0x%016llx\n", big_no);

    printk(KERN_ERR"LENGTH 0x%08x (%d)\n", desc->length, desc->length);
    printk(KERN_ERR"BYTES TRANSFERRED 0x%08x (%d)\n", desc->length, desc->length);

    printk(KERN_ERR"STATUS 0x%08x\n", desc->status);
    printk(KERN_ERR" %s %02x\n",
           (desc->status & (1<<8)) ? "EARLY-TERM" :".",
           (desc->status & 0xff));

    printk(KERN_ERR"BURST-SEQUENCE 0x%08x\n", desc->burst_sequence);
    printk(KERN_ERR" write burst count %d, read burst count %d, sequence no %d\n",
           (desc->burst_sequence & 0xff000000) >> 24,
           (desc->burst_sequence & 0x00ff0000) >> 16,
           (desc->burst_sequence & 0x0000ffff));

    printk(KERN_ERR"STRIDE 0x%08x\n",desc->stride);
    printk(KERN_ERR" write stride %d, read stride %d\n",
           (desc->stride & 0xffff0000) >> 16,
           (desc->stride & 0x0000ffff));

    printk(KERN_ERR"CONTROL 0x%08x\n", desc->control);
    printk(KERN_ERR" %s %s %s error-enable 0x%02x, %s %s %s %s %s %s %s transmit-channel %d\n",
           desc->control & (1<<31) ? "GO" : ".",
           desc->control & (1<<30) ? "HW" : ".",
           desc->control & (1<<24) ? "EARLY DONE" : ".",
           (desc->control & 0x00ff0000) >> 16,
           desc->control & (1<<15) ? "EARLY TERM IRQ" : ".",
           desc->control & (1<<14) ? "TRANS DONE IRQ" : ".",
           desc->control & (1<<12) ? "END EOP" : ".",
           desc->control & (1<<11) ? "PARK WRITES" : ".",
           desc->control & (1<<10) ? "PARK READS" : ".",
           desc->control & (1<<9) ? "GEN EOP" : ".",
           desc->control & (1<<8) ? "GEN SOP" : ".",
           desc->control & 0x000000ff);
}

/**
 * Dump registers and descriptor chains and anything else useful to the
 * kernel log
 *
 * @param adma driver structure
 */
static void crazy_dump_debug(struct altr_dma_dev* adma) {
    u32 reg;
    u64 hi;
    minit_dma_extdesc_t* desc;

    static char* response_port_strings[] = {
        "memory-mapped",
        "streaming",
        "disabled",
        "reserved!"
    };
    static char* transfer_type_strings[] = {
        "full word",
        "aligned",
        "unaligned",
        "reserved!"
    };
    /* dump core */
    printk(KERN_ERR"mSGDMA core\n");
    reg = readl(adma->msgdma_base + MSGDMA_STATUS);
    printk(KERN_ERR"STATUS = 0x%08x\n",reg);
    printk(KERN_ERR" %s %s %s %s %s %s %s %s %s %s\n",
           reg & MSGDMA_STATUS_IRQ ? "IRQ" : ".",
           reg & MSGDMA_STATUS_STOP_EARLY ? "STOP-EARLY" : ".",
           reg & MSGDMA_STATUS_STOP_ERROR ? "STOP-ERROR" : ".",
           reg & MSGDMA_STATUS_RESETTING ? "RESETTING" : ".",
           reg & MSGDMA_STATUS_STOPPED ? "STOPPED" : ".",
           reg & MSGDMA_STATUS_RESP_FULL ? "RESP-FULL" : ".",
           reg & MSGDMA_STATUS_RESP_EMPTY ? "RESP-EMPTY" : ".",
           reg & MSGDMA_STATUS_DESC_FULL ? "DESC-FULL" : ".",
           reg & MSGDMA_STATUS_DESC_EMPTY ? "DESC-EMPTY" : ".",
           reg & MSGDMA_STATUS_BUSY ? "BUSY" : ".");

    reg = readl(adma->msgdma_base + MSGDMA_CONTROL);
    printk(KERN_ERR"CONTROL = 0x%08x",reg);
    printk(KERN_ERR" %s %s %s %s %s %s\n",
           reg & MSGDMA_CTRL_STOP_DESC ? "STOP-DESC" : ".",
           reg & MSGDMA_CTRL_IRQ_MASK ? "IRQ-EN" : ".",
           reg & MSGDMA_CTRL_STOP_EARLY ? "STOP-EARLY" : ".",
           reg & MSGDMA_CTRL_STOP_ERROR ? "STOP-ERR" : ".",
           reg & MSGDMA_CTRL_RESET_DISPATCH ? "RESET-DISP" : ".",
           reg & MSGDMA_CTRL_STOP_DISPATCH ? "STOP-DISP" : ".");

    reg = readl(adma->msgdma_base + MSGDMA_RW_LEVEL);
    printk(KERN_ERR"MSGDMA_RW_LEVEL = 0x%08x\n",reg);
    printk(KERN_ERR" Write level = %d, Read level = %d\n",
           (reg & MSGDMA_WRITE_LEVEL_MASK) >> MSGDMA_WRITE_LEVEL_SHIFT,
           (reg & MSGDMA_READ_LEVEL_MASK) >> MSGDMA_READ_LEVEL_SHIFT);

    reg = readl(adma->msgdma_base + MSGDMA_RES_LEVEL);
    printk(KERN_ERR"MSGDMA_RES_LEVEL = 0x%08x\n",reg);
    printk(KERN_ERR" Response level = %d\n",
           (reg & MSGDMA_RESP_LEVEL_MASK) >> MSGDMA_RESP_LEVEL_SHIFT);

    reg = readl(adma->msgdma_base + MSGDMA_SEQ_NO);
    printk(KERN_ERR" Write sequence no = %d, Read sequence no = %d\n",
           (reg & MSGDMA_WRITE_SEQ_MASK) >> MSGDMA_WRITE_SEQ_SHIFT,
           (reg & MSGDMA_READ_SEQ_MASK) >> MSGDMA_READ_SEQ_SHIFT);

    reg = readl(adma->msgdma_base + MSGDMA_CONFIG_1);
    printk(KERN_ERR"MSGDMA_CONFIG_1 = 0x%08x\n",reg);
    printk(KERN_ERR" burst en %s, burst wrapping support %s, channel enable %s\n",
           reg & (1 << 0) ? "yes" : "no",
           reg & (1 << 1) ? "yes" : "no",
           reg & (1 << 2) ? "yes" : "no");
    printk(KERN_ERR" channel width %d, data fifo depth %d data width %d\n",
           ((reg & 0x00000038) >> 3) + 1,
           1 << (((reg & 0x000003c0) >> 6) + 4),
           1 << (((reg & 0x00001c00) >> 10) + 3) );
    printk(KERN_ERR" desc'r fifo depth %d, dma mode = %s to %s\n",
           1 << (((reg & 0x0000e000) >> 13) + 3),
           (reg & 0x00010000) >> 16 ? "stream" : "memory",
           (reg & 0x00020000) >> 17 ? "stream" : "memory");
    printk(KERN_ERR" enhanced features %s, error enable %s\n",
           reg & (1 << 18) ? "yes" : "no",
           reg & (1 << 19) ? "yes" : "no");
    printk(KERN_ERR" error width %d, max burst count %d, max transfer length %d\n",
           ((reg & 0x00700000) >> 20) + 1,
           1 << (((reg & 0x07800000) >> 23) + 1),
           1 << (((reg & 0xf8000000) >> 27) + 10) );

    reg = readl(adma->msgdma_base + MSGDMA_CONFIG_2);
    printk(KERN_ERR"MSGDMA_CONFIG_2 = 0x%08x\n",reg);
    printk(KERN_ERR" stride enable %s, max stride %d",
           reg & 0x00000001 ? "yes" : "no",
           ((reg & 0x0000fff7) >> 1) + 1);
    printk(KERN_ERR" packet enable %s, prefetcher enable %s, programmabe burst enable %s\n",
           reg & (1 << 16) ?"yes" : "no",
           reg & (1 << 17) ?"yes" : "no",
           reg & (1 << 18) ?"yes" : "no");
    printk(KERN_ERR" response port %s, transfer type %s acceses\n",
           response_port_strings[(reg & 0x000c0000) >> 19],
           transfer_type_strings[(reg & 0x00300000) >> 21]);

    printk(KERN_ERR"Descriptor Prefetcher core\n");
    reg = readl(adma->prefetcher_base + PRE_CONTROL);
    printk(KERN_ERR"PRE_CONTROL = 0x%08x\n",reg);
    printk(KERN_ERR" %s %s %s %s %s\n",
           reg & (1 << 4) ? "PARK_MODE" : ".",
           reg & (1 << 3) ? "IRQ_EN" : ".",
           reg & (1 << 2) ? "RESET" : ".",
           reg & (1 << 1) ? "POLL_EN" : ".",
           reg & (1 << 0) ? "RUN" : ".");


    hi = reg = readl(adma->prefetcher_base + PRE_NEXT_DESC_LO);
    printk(KERN_ERR"PRE_NEXT_DESC_LO = 0x%08x\n",reg);
    reg = readl(adma->prefetcher_base + PRE_NEXT_DESC_HI);
    printk(KERN_ERR"PRE_NEXT_DESC_HI = 0x%08x\n",reg);
    printk(KERN_ERR" next descriptor address 0x%016llx\n", (hi << 32) + reg);

    reg = readl(adma->prefetcher_base + PRE_DESC_POLL_FREQ);
    printk(KERN_ERR"PRE_DESC_POLL_FREQ = 0x%08x\n",reg);
    printk(KERN_ERR" poll frequency %d cycles\n",
           reg & 0x0000ffff);

    reg = readl(adma->prefetcher_base + PRE_STATUS);
    printk(KERN_ERR"PRE_STATUS = 0x%08x\n",reg);
    printk(KERN_ERR" %s\n",
           (reg & 0x00000001) ? "IRQ": ".");

    // dump descriptor chains.
    if (adma->transfer_on_hardware) {
        desc = adma->transfer_on_hardware->descriptor;
        while (desc) {
            dump_descriptor(desc);
            desc = desc->next_desc_virt;
        }
    }
}


static inline struct transfer_job_s* pop_job(struct list_head* list, spinlock_t* lock)
{
    unsigned long flags;
    struct transfer_job_s* job = NULL;
    spin_lock_irqsave(lock, flags);
    if (!list_empty(list)) {
        job = list_first_entry(list, struct transfer_job_s, list);
        list_del(&job->list);
    }
    spin_unlock_irqrestore(lock, flags);
    return job;
}

static inline void push_job(struct list_head* list, spinlock_t* lock, struct transfer_job_s* job)
{
    unsigned long flags;
    spin_lock_irqsave(lock, flags);
    list_add_tail(list, &job->list);
    spin_unlock_irqrestore(lock, flags);
}

/**
 * alloc_sg_table_from_pages: this is copied from sg_alloc_table_from_pages, but
 * this version obeys the max transfer size.
 *
 * Allocate and initialize an sg table from an array of pages
 *
 * @param sgt:  The sg table header to use
 * @param pages:    Pointer to an array of page pointers
 * @param n_pages:  Number of pages in the pages array
 * @param offset:     Offset from start of the first page to the start of a buffer
 * @param size:       Number of valid bytes in the buffer (after offset)
 * @param gfp_mask: GFP allocation mask
 *
 * Allocate and initialize an sg table from a list of pages. Contiguous
 * ranges of the pages are squashed together into scatterlist nodes that are
 * under the DMA_MAX_SIZE.
 * A user may provide an offset at a start and a size of valid data in a buffer
 * specified by the page array. The returned sg table is released by
 * sg_free_table.
 *
 * @return 0 on success, negative error on failure
 */
static int alloc_sg_table_from_pages(struct sg_table *sgt,
    struct page **pages, unsigned int n_pages,
    unsigned long offset, unsigned long size,
    unsigned long max_transfer_size,
    gfp_t gfp_mask)
{
    unsigned int chunks;
    unsigned long chunk_size;
    unsigned int i;
    unsigned int cur_page;
    int ret;
    struct scatterlist *s;

    /* compute number of contiguous chunks, under our size limit */
    chunks = 1;
    chunk_size = PAGE_SIZE - offset;
    for (i = 1; i < n_pages; ++i) {
        // if not contiguous or chunk too big
        if ((page_to_pfn(pages[i]) != page_to_pfn(pages[i - 1]) + 1) ||
            ((chunk_size + PAGE_SIZE) > max_transfer_size))
        {
            ++chunks;
            chunk_size = 0;
        }
        chunk_size += PAGE_SIZE;
    }
    VPRINTK("%d pages in %d chunks/sg-elements\n",n_pages, chunks);

    ret = sg_alloc_table(sgt, chunks, gfp_mask);
    if (unlikely(ret))
        return ret;

    /* merging chunks and putting them into the scatterlist */
    cur_page = 0;
    chunk_size = PAGE_SIZE - offset;
    for_each_sg(sgt->sgl, s, sgt->orig_nents, i) {
        unsigned int j;
        VPRINTK("sg-elem %d\n",i);

        /* look for the end of the current chunk. within size limit*/
        for (j = cur_page + 1; j < n_pages; ++j) {
            VPRINTK("%lx vs %lx\n", page_to_pfn(pages[j]) ,page_to_pfn(pages[j-1])+PAGE_SIZE);
            if ((page_to_pfn(pages[j]) != page_to_pfn(pages[j - 1]) + 1) ||
                ((chunk_size + PAGE_SIZE) > max_transfer_size))
            {
                VPRINTK("break between pages %d..%d size %ld (%lx)\n",j-1,j, chunk_size, chunk_size);
                break;
            }
            chunk_size += PAGE_SIZE;
            VPRINTK("adding page %d size %ld (%lx)\n",j, chunk_size, chunk_size);
        }

        VPRINTK("sg_elem %d at %p size %ld (%lx), offset %lx\n", i, page_address(pages[cur_page]), min(size, chunk_size), min(size, chunk_size), offset);
        sg_set_page(s, pages[cur_page], min(size, chunk_size), offset);
        size -= chunk_size;
        chunk_size = PAGE_SIZE;
        offset = 0;
        cur_page = j;
    }

    return 0;
}

static inline void descriptor_set_phys(u32* hi, u32* lo, dma_addr_t desc_phys)
{
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
    *hi = (u32)(desc_phys >> 32);
    *lo = (u32)(desc_phys & 0x00000000ffffffff);
#else
    *hi = 0;
    *lo = desc_phys;
#endif
}

static inline dma_addr_t descriptor_get_phys(u32* hi, u32* lo)
{
    dma_addr_t ret;

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
    ret = *hi;
    ret <<= 32;
    ret |= *lo;
#else
    ret = *lo;
#endif
    return ret;
}

static int dma_busy(struct altr_dma_dev* adma)
{
    return readl(adma->msgdma_base + MSGDMA_CONTROL) & MSGDMA_STATUS_BUSY;
}

static void reset_dma_hardware(struct altr_dma_dev* adma)
{
    // reset prefetcher and wait for hardware to clear reset
    writel(PRE_CTRL_RESET, adma->prefetcher_base + PRE_CONTROL);
    while(readl(adma->prefetcher_base + PRE_CONTROL) | PRE_CTRL_RESET)
        ;

    // reset mSGDMA/dispatcher core and wait for the hardware to clear reset
    writel(MSGDMA_CTRL_RESET_DISPATCH, adma->msgdma_base + MSGDMA_CONTROL);
    while(readl(adma->msgdma_base + MSGDMA_STATUS) | MSGDMA_STATUS_RESETTING)
        ;

}

static void start_transfer_unlocked(struct altr_dma_dev* adma)
{
    struct transfer_job_s* job;

    // if the hardware is busy or we're already processing a job, exit
    if (dma_busy(adma) ||
        adma->transfer_on_hardware ||
        list_empty(&adma->transfers_ready_for_hardware))
    {
        return;
    }

    job = list_first_entry(&job->list, struct transfer_job_s, list);
    list_del(&job->list);
    adma->transfer_on_hardware = job;

    // load the descriptor chain into the prefetcher core
    descriptor_set_phys(
        adma->prefetcher_base + PRE_NEXT_DESC_HI,
        adma->prefetcher_base + PRE_NEXT_DESC_LO,
        job->descriptor_phys);
    wmb();

    // start the prefetcher core
    writel(ALTERA_DMA_PRE_START, adma->prefetcher_base + PRE_CONTROL);
}

static long submit_transfer_to_hw_or_queue(struct altr_dma_dev* adma, struct transfer_job_s* job)
{
    unsigned long flags;

    // lock something
    spin_lock_irqsave(&adma->hardware_lock, flags);

    // if the queue isn't empty then add to back of queue
    list_add_tail(&job->list, &adma->transfers_ready_for_hardware);
    start_transfer_unlocked(adma);

    // unlock something
    spin_unlock_irqrestore(&adma->hardware_lock, flags);
    return -1;
}

static void free_descriptor_list(struct altr_dma_dev* adma, struct transfer_job_s* job)
{
    minit_dma_extdesc_t* desc = job->descriptor;
    dma_addr_t desc_phys = job->descriptor_phys;

    while (desc) {
        minit_dma_extdesc_t* next = desc->next_desc_virt;
        dma_addr_t next_phys = descriptor_get_phys(&desc->next_desc_hi_phys, &desc->next_desc_lo_phys);
        dma_pool_free(adma->descriptor_pool, desc, desc_phys);
        desc = next;
        desc_phys = next_phys;
    }
    job->descriptor = NULL;
    job->descriptor_phys = 0;
}

static long create_descriptor_list(struct altr_dma_dev* adma, struct transfer_job_s* job)
{
    struct scatterlist* sg_elem;
    int sg_index;
    int rc;

    minit_dma_extdesc_t* prev_desc = NULL;

    // allocate DMA descriptors for transfer from dma-pool
    for_each_sg(job->sgt.sgl, sg_elem, job->sgt.nents, sg_index) {
        minit_dma_extdesc_t* desc;
        dma_addr_t desc_phys;
        desc = dma_pool_alloc(adma->descriptor_pool, GFP_KERNEL, &desc_phys);
        if (!desc) {
            // oh poop!
            rc = -ENOMEM;
            goto err_free_descriptors;
        }
        // fill in transfer details
        descriptor_set_phys(&desc->read_hi_phys, &desc->read_lo_phys, 0);
        descriptor_set_phys(&desc->write_hi_phys, &desc->write_lo_phys, sg_elem->dma_address);
        desc->length = sg_elem->length;
        desc->bytes_transferred = 0;
        desc->status = 0;
        desc->burst_sequence = 0;
        desc->stride = 0x00010000; //write stride 1 read stride 0
        desc->control = ALTERA_DMA_DESC_CONTROL_NOT_END;

        // make the head or previous descriptor link to this one
        if (sg_index == 0) {
            job->descriptor = desc;
            job->descriptor_phys = desc_phys;
        } else {
            prev_desc->next_desc_virt = desc;
            descriptor_set_phys(
                        &prev_desc->next_desc_hi_phys,
                        &prev_desc->next_desc_lo_phys,
                        desc_phys);
        }

        prev_desc = desc;
    }
    prev_desc->control = ALTERA_DMA_DESC_CONTROL_END;

    // should be ready for hardware
    rc = submit_transfer_to_hw_or_queue(adma, job);
    if (rc) {
        goto err_free_descriptors;
    }
    return 0;

err_free_descriptors:
    free_descriptor_list(adma, job);

    return rc;
}

/**
 * This performs the DMA mapping to get DMA addresses for the memory buffer
 * supplied. It then passes the transfer along to the next stage.
 *
 * @brief Queue the transfer for fetching data from the hardware
 * @param adma deviec structure
 * @param transfer Information about the transfer, field validation will be performed
 * in this function
 * @return < 0 if an error.
 */
long queue_data_transfer(struct altr_dma_dev* adma, struct minit_data_transfer_s* transfer)
{
    long rc = 0;
    struct page** pages;
    int actual_pages;
    int nents;
    unsigned long pg_start;
    unsigned long pg_start_offset;
    unsigned long pg_end;
    unsigned int no_pages;

    // create job structure, freed in get_completed_data_transfers, or cancel_data_transfer
    struct transfer_job_s* job = kzalloc(sizeof(struct transfer_job_s), GFP_KERNEL);
    if (!job) {
        DPRINTK("Failed to allocate memory for transfer_job\n");
        return -ENOMEM;
    }
    job->buffer = transfer->buffer;
    job->buffer_size = transfer->buffer_size;
    job->transfer_id = transfer->transfer_id;
    job->signal_number = transfer->signal_number;
    job->pid = transfer->pid;

    // convert to scaterlist and do dma mapping
    pg_start = (unsigned long)job->buffer & PAGE_MASK;
    pg_start_offset = (unsigned long)job->buffer & ~PAGE_MASK;
    pg_end = PAGE_ALIGN((unsigned long)(job->buffer + job->buffer_size));
    no_pages = (pg_end - pg_start) >> PAGE_SHIFT;

    DPRINTK("DMA transfer, usespace buffer %p len %d\n",
        job->buffer,
        job->buffer_size);

    // Alocate memory for page list
    pages = kzalloc(sizeof(struct page*) * no_pages, GFP_KERNEL);
    if (!pages) {
        printk(KERN_ERR"Couldn't allocate memory for page-table\n");
        rc = -ENOMEM;
        goto err;
    }
    VPRINTK("page list at %p",pages);

    // get page entries for buffer pages (write to pages)
    actual_pages = get_user_pages_fast(pg_start, no_pages, 1, pages);
    VPRINTK("mapped %d pages from userspace\n",actual_pages);
    VPRINTK("start page addr %lx, end page addr %lx, start offset %lx\n",
        pg_start, pg_end, pg_start_offset);

    // check we got all the pages of the buffer
    if ( actual_pages != no_pages) {
        printk(KERN_ERR"couldn't get all the buffer's pages (%d vs %d)\n",actual_pages, no_pages);
        rc = -ENOMEM;
        goto err_free_pages;
    }

    // create scatterlist
    if (alloc_sg_table_from_pages(
        &job->sgt,
        pages,
        actual_pages,
        pg_start_offset,
        job->buffer_size,
        adma->max_transfer_size,
        GFP_KERNEL))
    {
        printk(KERN_ERR"Couldn't allocate memory for scatterlist\n");
        rc = -ENOMEM;
        goto err_free_pages;
    }

    // map dma
    nents = pci_map_sg( adma->pci_device, job->sgt.sgl, job->sgt.nents, DMA_FROM_DEVICE);
    if (!nents) {
        printk(KERN_ERR"Error mapping dma to bus addresses\n");
        rc = -ENOMEM;
        goto err_free_scatterlist;
    }
    VPRINTK("Mapped scattelist to %d transfers\n", nents);

    // queue for creating descriptor list, return if no error
    rc = create_descriptor_list(adma, job);
    if (rc) {
        // success
        goto err_unmap;
    }


    return 0;
err_unmap:
    // unmap dma
    VPRINTK("unmap sg\n");
    pci_unmap_sg(adma->pci_device, job->sgt.sgl, job->sgt.nents, DMA_FROM_DEVICE );
err_free_scatterlist:
    // free scatterlist
    VPRINTK("free scatterlist table\n");
    sg_free_table(&job->sgt);
err_free_pages:
    // release any allocated pages, no writing done yet.
    while (actual_pages > 0) {
        struct page* page;
        --actual_pages;
        page = pages[actual_pages];
        VPRINTK("release page %d %p -> %p\n",actual_pages, page, page_address(page) );

        // release it back into the wild
        put_page(page);
    }
    kfree(pages);
err:
    return rc;
}

/**
 * @brief get completed data transfers
 * @param adma driver object
 * @param max_elem size (in number of elements) of statuses
 * @param statuses details of completed transfers written here
 * @return number of completed transfers found
 *
 * Returns details about completed transfers and frees resources associated with the transfer
 */
u32 get_completed_data_transfers(struct altr_dma_dev* adma, u32 max_elem, struct minit_transfer_status_s* statuses)
{
    u32 i;
    for (i = 0 ; i < max_elem ; ++i) {
        /// @todo need to only pop job if pid matches
        struct transfer_job_s* job = pop_job(&adma->transfers_done, &adma->done_lock);
        if (!job) {
            break;
        }
        statuses[i].transfer_id = job->transfer_id;
        statuses[i].status = job->status;

        // transfer probably done, data is in memory, status recorded, can now
        // destroy the job object
        kfree(job);
    }

    // 'i' should now be the number of jobs or max_elem
    return i;
}

static struct transfer_job_s* find_job_to_cancel(struct altr_dma_dev* adma, u32 transfer_id)
{
    struct transfer_job_s* job;
    unsigned long flags;

    spin_lock_irqsave(&adma->hardware_lock, flags);
    list_for_each_entry(job, &adma->transfers_ready_for_hardware, list) {
        if (job->transfer_id == transfer_id) {
            list_del(&job->list);
            return job;
        }
    }
    if (adma->transfer_on_hardware->transfer_id == transfer_id) {
        job = adma->transfer_on_hardware;
        reset_dma_hardware(adma);
        return job;
    }
    list_for_each_entry(job, &adma->post_hardware, list) {
        if (job->transfer_id == transfer_id) {
            list_del(&job->list);
            return job;
        }
    }
    spin_unlock_irqrestore(&adma->hardware_lock,flags);

    return NULL;
}

/**
 * @brief cancel_data_transfer
 * @param adma
 * @param transfer_id
 * @return
 *
 * proceed through the queues looking for the transfer
 */
long cancel_data_transfer(struct altr_dma_dev* adma, u32 transfer_id)
{
    unsigned long flags;
    struct transfer_job_s* job;

    job = find_job_to_cancel(adma, transfer_id);
    if (job) {
        // free descriptors and unmap dma
        free_descriptor_list(adma, job);
        pci_unmap_sg(adma->pci_device, job->sgt.sgl, job->sgt.nents, DMA_FROM_DEVICE);
        kfree(job);
        return 0;
    }

    // may have already completed, but the userspace app still wants to cancel
    spin_lock_irqsave(&adma->done_lock, flags);
    list_for_each_entry(job, &adma->transfers_ready_for_hardware, list) {
        if (job->transfer_id == transfer_id) {
            list_del(&job->list);
            kfree(job);
            spin_unlock_irqrestore(&adma->done_lock, flags);
            return 0;
        }
    }
    spin_unlock_irqrestore(&adma->done_lock, flags);

    // not found
    return -1;
}

/**
 * @brief interrupt handler, was it us?
 *
 * If the irq came from dma hardware, clear irq and start bottom-half
 */
static irqreturn_t dma_isr_quick(int irq_no, void* dev)
{
    irqreturn_t ret = IRQ_HANDLED;
    struct altr_dma_dev* adma = (struct altr_dma_dev*)dev;
    if (!adma) {
        // uh oh!
        return IRQ_NONE;
    }

    // check the hardware, if the interrupt was us then run bh
    if (readl(adma->prefetcher_base + PRE_STATUS) & PRE_STATUS_IRQ) {
        writel(PRE_STATUS_IRQ, adma->prefetcher_base + PRE_STATUS);
        ret = IRQ_WAKE_THREAD;
    }

    return ret;
}

/**
 * @brief bottom-half interrupt handler, remove done transfer and start
 * next transfer.
 *
 * Queues the complete transfer to have the resources freed, etc done in a
 * workqueue
 */
static irqreturn_t dma_isr(int irq_no, void* dev)
{
    struct altr_dma_dev* adma = (struct altr_dma_dev*)dev;
    if (!adma) {
        // uh oh!
        DPRINTK("Bottom half, called without device sturcture\n");
        return IRQ_NONE;
    }

    spin_lock(&adma->hardware_lock);
    // is there a transfer complete
    while (!dma_busy(adma) && adma->transfer_on_hardware) {
        u32 status;
        struct transfer_job_s* job = adma->transfer_on_hardware;
        adma->transfer_on_hardware = NULL;

        // check for error, reset core ?
        status = readl(adma->msgdma_base + MSGDMA_STATUS);
        job->status = status;
        if (status & (MSGDMA_STATUS_STOP_EARLY| MSGDMA_STATUS_STOP_ERROR|MSGDMA_STATUS_STOPPED)) {
            crazy_dump_debug(adma);
            reset_dma_hardware(adma);
        }

        list_add_tail(&adma->post_hardware, &job->list);

        // next waiting transfer
        start_transfer_unlocked(adma);

        // start the workqueue to inform the user
        queue_work(adma->finishing_queue, &adma->finishing_work);
    }

    spin_unlock(&adma->hardware_lock);
    return IRQ_HANDLED;
}

static void send_signal(const int signal_number, const int pid)
{
    if (signal_number) {
        int rc;
        struct task_struct* task;

        rcu_read_lock();
        task = pid_task(find_vpid( pid ), PIDTYPE_PID);
        rcu_read_unlock();
        rc = send_sig(signal_number, task, 0);
        if (rc < 0) {
            DPRINTK("error sending signal\n");
        }
    }
}

/**
 * @brief post_transfer houskeeping and signal to userspace
 * @param work embedded in the device structure.
 *
 * Free descriptors, do the dma unmapping and send a signal to the user process
 * associated with the transfer
 */
static void post_transfer(struct work_struct *work)
{
    struct transfer_job_s* job;
    struct altr_dma_dev* adma = (struct altr_dma_dev*)
            container_of(work, struct altr_dma_dev, finishing_work);

    while ((job = pop_job(&adma->post_hardware, &adma->hardware_lock)) != NULL) {
        free_descriptor_list(adma,job);
        pci_unmap_sg(adma->pci_device, job->sgt.sgl, job->sgt.nents, DMA_FROM_DEVICE);

        // move to done
        push_job(&adma->transfers_done, &adma->done_lock, job);
        send_signal(job->signal_number,job->pid);
    }
}

int altera_sgdma_probe(struct minit_device_s* mdev) {
    /// @TODO confirm correct version of hardware

    // construct dma coordination structure and link with driver
    struct altr_dma_dev* adma = kzalloc(sizeof(struct altr_dma_dev), GFP_KERNEL);
    if (!adma) {
        return -ENOMEM;
    }
    adma->msgdma_base = mdev->ctrl_bar + ASIC_HS_DMA_BASE;
    adma->prefetcher_base = mdev->ctrl_bar + ASIC_HS_DMA_PREF_BASE;
    adma->pci_device = mdev->pci_device;
    adma->max_transfer_size = 2048; /// @TODO get this from the hardware registers

    mdev->dma_isr_quick = dma_isr_quick;
    mdev->dma_isr = dma_isr;
    mdev->dma_dev = adma;

    // allocate a bunch of descriptors in coherant memory
    ///@TODO replace number of descriptors (128) with some sort of global constant
    adma->descriptor_pool = dmam_pool_create("MinION DMA", &mdev->pci_device->dev, sizeof(minit_dma_extdesc_t) * 128, 4, 0);
    if (!adma->descriptor_pool) {
        DPRINTK("Unable to allocate a pool of memory for dma descriptors\n");
        return -ENOMEM;
    }

    // create workqueue and work to do the post-transfer unmapping and callbacks
    adma->finishing_queue = create_singlethread_workqueue("minit-post-transfer");
    INIT_WORK(&adma->finishing_work, post_transfer);

    crazy_dump_debug(adma);
    return -1;
}

void altera_sgdma_remove(struct minit_device_s* mdev) {
    struct altr_dma_dev* adma = mdev->dma_dev;
    if (!adma) {
        return;
    }

    kfree(adma);
    mdev->dma_isr_quick = NULL;
    mdev->dma_isr = NULL;
    mdev->dma_dev = NULL;
}
