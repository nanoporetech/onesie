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
    if (!desc) {
        printk(KERN_ERR"NULL!\n");
        return;
    }

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
    printk(KERN_ERR"BYTES TRANSFERRED 0x%08x (%d)\n", desc->bytes_transferred, desc->bytes_transferred);

    printk(KERN_ERR"STATUS 0x%08x\n", desc->status);
    printk(KERN_ERR" %s Error code %02x\n",
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
    printk(KERN_ERR"driver_ref (job) %p\n",desc->driver_ref);
    printk(KERN_ERR"next desc virt %p\n",desc->next_desc_virt);
}

/**
 * @brief Dump a job and its DMA descriptors to kernel log
 * @param job
 */
static void dump_job(struct transfer_job_s* job)
{
    minit_dma_extdesc_t* desc;

    if (!job) {
        printk(KERN_ERR"NULL!\n");
        return;
    }

    printk(KERN_ERR"job structure at %p\n",job);
    printk(KERN_ERR" buffer %p\n",job->buffer);
    printk(KERN_ERR" buffer_size 0x%08x (%d)\n",job->buffer_size,job->buffer_size);
    printk(KERN_ERR" transfer_id %d\n",job->transfer_id);
    printk(KERN_ERR" signal_number %d\n",job->signal_number);
    printk(KERN_ERR" pid %d\n",job->pid);
    printk(KERN_ERR" scatter-table\n"); // todo
    printk(KERN_ERR" status %d\n",job->status);
    printk(KERN_ERR" file %p\n",job->file);
    printk(KERN_ERR" descriptor physical address 0x%016llx\n", job->descriptor_phys);
    printk(KERN_ERR" descriptors:\n");
    desc = job->descriptor;
    while (desc) {
        dump_descriptor(desc);
        desc = desc->next_desc_virt;
    }
}

/**
 * Dump registers and descriptor chains and anything else useful to the
 * kernel log
 *
 * @param adma driver structure
 */
static void dump_dma_registers(struct altr_dma_dev* adma) {
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
    if (!adma) {
        printk(KERN_ERR"struct altr_dma_dev in NULL!\n");
        return;
    }

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
    if (0) {
    reg = readl(adma->msgdma_base + MSGDMA_CONFIG_1);
    printk(KERN_ERR"MSGDMA_CONFIG_1 = 0x%08x\n",reg);
    printk(KERN_ERR" burst en %s, burst wrapping support %s, channel enable %s\n",
           reg & (1 << 0) ? "yes" : "no",
           reg & (1 << 1) ? "yes" : "no",
           reg & (1 << 2) ? "yes" : "no");
    printk(KERN_ERR" channel width %d, data fifo depth %d, data width %d\n",
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
    }
    printk(KERN_ERR"Descriptor Prefetcher core\n");
    reg = readl(adma->prefetcher_base + PRE_CONTROL);
    printk(KERN_ERR"PRE_CONTROL = 0x%08x\n",reg);
    printk(KERN_ERR" %s %s %s %s %s\n",
           reg & (1 << 4) ? "PARK_MODE" : ".",
           reg & (1 << 3) ? "IRQ_EN" : ".",
           reg & (1 << 2) ? "RESET" : ".",
           reg & (1 << 1) ? "POLL_EN" : ".",
           reg & (1 << 0) ? "RUN" : ".");


    reg = readl(adma->prefetcher_base + PRE_NEXT_DESC_LO);
    printk(KERN_ERR"PRE_NEXT_DESC_LO = 0x%08x\n",reg);
    hi = readl(adma->prefetcher_base + PRE_NEXT_DESC_HI);
    printk(KERN_ERR"PRE_NEXT_DESC_HI = 0x%08x\n",hi);
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

/**
 * @brief dump the altr_dma_dev structure and its children
 */
static void crazy_dump_debug(struct altr_dma_dev* adma)
{
    struct transfer_job_s* job;

    if (!adma) {
        printk(KERN_ERR"struct altr_dma_dev in NULL!\n");
        return;
    }

    printk(KERN_ERR"msgdma base %p\n",adma->msgdma_base);
    printk(KERN_ERR"prefetcher base %p\n",adma->prefetcher_base);
    dump_dma_registers(adma);

    printk(KERN_ERR"pci_device %p\n",adma->pci_device);
    printk(KERN_ERR"max transfer size 0x%x (%d)\n",adma->max_transfer_size,adma->max_transfer_size);
    printk(KERN_ERR"hardware lock %s\n", spin_is_locked(&adma->hardware_lock) ? "locked" : "unlocked" ); /// @todo
    printk(KERN_ERR"transfers ready for hardware\n");
    list_for_each_entry(job, &adma->transfers_ready_for_hardware, list) {
        dump_job(job);
    }
    printk(KERN_ERR"transfers on hardware\n");
    dump_job(adma->transfer_on_hardware);
    printk(KERN_ERR"post hardware transfers\n");
    list_for_each_entry(job, &adma->post_hardware, list) {
        dump_job(job);
    }
    printk(KERN_ERR"done lock %s\n", spin_is_locked(&adma->hardware_lock) ? "locked" : "unlocked"); /// @todo
    printk(KERN_ERR"done transfers\n");
    list_for_each_entry(job, &adma->transfers_done, list) {
        dump_job(job);
    }
    printk(KERN_ERR"descriptor pool %p\n",adma->descriptor_pool);
    printk(KERN_ERR"finishing queue %p\n",adma->finishing_queue);
}

/**
 * @brief extract the maximum transfer size from the hardware config registers
 * @param adma
 * @return maximum transfer-size in bytes
 */
static unsigned int get_max_transfer_size(struct altr_dma_dev* adma)
{
    return 1 << (((READL(adma->msgdma_base + MSGDMA_CONFIG_1) & 0xf8000000) >> 27) + 10);
}

/**
 * @brief Pop a job from the end of a list using the lock provided
 * @param list List of jobs
 * @param lock This will be taken with irqsave whilst interacting with the list
 * @return The job on the end of the list or null if the list was empty
 */
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

/**
 * @brief push_job Push a job onto the start of a list using the lock provided
 * @param list List of jobs
 * @param lock This will be taken with irqsave whilst interacting with the list
 * @param job Job to push onto the list
 */
static inline void push_job(struct list_head* list, spinlock_t* lock, struct transfer_job_s* job)
{
    unsigned long flags;
    spin_lock_irqsave(lock, flags);
    list_add_tail(&job->list, list);
    spin_unlock_irqrestore(lock, flags);
}

/**
 * @brief write a 64/32-bit dma_addr_t into a 64-bit DMA address
 * split into two 32-bit high and low fields
 * @param hi pointer to upper 32-bit field
 * @param lo pointer to lower 32-bit field
 * @param desc_phys dma address
 */
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

/**
 * @brief read two 32-bit high and low fields into a 32/64 bit dma_addr_t
 * @param hi pointer to upper 32-bit field
 * @param lo pointer to lower 32-bit field
 * @return 32 or 64-bit dma address
 */
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

/**
 * @brief is DMA hardware busy
 * @param adma
 * @return 0 = idle
 */
static int dma_busy(struct altr_dma_dev* adma)
{
    return READL(adma->msgdma_base + MSGDMA_CONTROL) & MSGDMA_STATUS_BUSY;
}

/**
 * @brief reset dma hardware. Call holding hardware_lock
 * @param adma
 */
static void reset_dma_hardware(struct altr_dma_dev* adma)
{
    /// @todo should not wait forever for resets to clear, timout with some
    /// sort of error and mark the hardware as broken so user-space apps can
    /// save their data.

    // reset prefetcher and wait for hardware to clear reset
    WRITEL(PRE_CTRL_RESET, adma->prefetcher_base + PRE_CONTROL);
    while(READL(adma->prefetcher_base + PRE_CONTROL) & PRE_CTRL_RESET)
        ;

    // reset mSGDMA/dispatcher core and wait for the hardware to clear reset
    WRITEL(MSGDMA_CTRL_RESET_DISPATCH, adma->msgdma_base + MSGDMA_CONTROL);
    while(READL(adma->msgdma_base + MSGDMA_STATUS) & MSGDMA_STATUS_RESETTING)
        ;

}

/**
 * @brief start the next transfer ready for hardware if idle. Call holding hardware_lock
 */
static void start_transfer_unlocked(struct altr_dma_dev* adma)
{
    struct transfer_job_s* job;

    VPRINTK("start_transfer_unlocked\n");
    // if the hardware is busy or we're already processing a job, exit
    if (dma_busy(adma) ||
        adma->transfer_on_hardware ||
        list_empty(&adma->transfers_ready_for_hardware))
    {
        VPRINTK("start_transfer_unlocked we already seem busy or nothing to do\n");
        return;
    }

    job = list_first_entry(&adma->transfers_ready_for_hardware, struct transfer_job_s, list);
    list_del(&job->list);
    adma->transfer_on_hardware = job;

    // load the descriptor chain into the prefetcher core
    descriptor_set_phys(
        adma->prefetcher_base + PRE_NEXT_DESC_HI,
        adma->prefetcher_base + PRE_NEXT_DESC_LO,
        job->descriptor_phys);
    wmb();

    // start the prefetcher core
    WRITEL(ALTERA_DMA_PRE_START, adma->prefetcher_base + PRE_CONTROL);
}

/**
 * @brief put a job on hardware or queue it
 * @param adma
 * @param job Has buffer converted to scatterlist, mapped and descriptors created
 *
 * will briefly touch the transfers_ready_for_hardware queue, even if empty
 */
static void submit_transfer_to_hw_or_queue(struct altr_dma_dev* adma, struct transfer_job_s* job)
{
    unsigned long flags;

    VPRINTK("submit_transfer_to_hw_or_queue\n");
    // lock something
    spin_lock_irqsave(&adma->hardware_lock, flags);

    // if the queue isn't empty then add to back of queue
    list_add_tail(&job->list, &adma->transfers_ready_for_hardware);
    start_transfer_unlocked(adma);

    // unlock something
    spin_unlock_irqrestore(&adma->hardware_lock, flags);
}

/**
 * @brief walk descriptor list freeing entries and returning resources to descritor pool
 * @param adma
 * @param job
 */
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

/**
 * @brief convert a mapped scatterlist to a dma-descriptors list
 * @param adma
 * @param job
 * @param nents the number of transfers in the scatter-list as returned by
 * pci_map_sg or dma_map_sg
 * @return error code or 0 on success
 *
 * This will honour the adma->max_transfer_size, splitting transfers into
 * multiple jobs if they're too big.
 */
static long create_descriptor_list(struct altr_dma_dev* adma, struct transfer_job_s* job, int nents)
{
    struct scatterlist* sg_elem;
    int sg_index;
    int descriptor_no = 0;
    int rc;

    minit_dma_extdesc_t* prev_desc = NULL;

    VPRINTK("create_descriptor_list\n");
    // allocate DMA descriptors for transfer from dma-pool
    for(sg_index = 0, sg_elem = job->sgt.sgl; sg_index < nents; sg_index++, sg_elem++) {
        unsigned int length = sg_dma_len(sg_elem);
        dma_addr_t dma_address = sg_dma_address(sg_elem);
        while (length) {
            minit_dma_extdesc_t* desc;
            dma_addr_t desc_phys;
            desc = dma_pool_alloc(adma->descriptor_pool, GFP_KERNEL, &desc_phys);
            VPRINTK("allocated memory for desciptor at %p (phys 0x%016llx)\n", desc, desc_phys);
            if (!desc) {
                DPRINTK("error, couldn't allocate dma descriptor\n");
                // oh poop!
                rc = -ENOMEM;
                goto err_free_descriptors;
            }
            // fill in transfer details
            descriptor_set_phys(&desc->read_hi_phys, &desc->read_lo_phys, 0);
            descriptor_set_phys(&desc->write_hi_phys, &desc->write_lo_phys, dma_address);
            desc->length = min(length, adma->max_transfer_size);
            length -= desc->length;
            dma_address += desc->length;
            desc->bytes_transferred = 0;
            desc->status = 0;
            desc->burst_sequence = 0;
            desc->stride = 0x00010000; //write stride 1 read stride 0
            desc->control = ALTERA_DMA_DESC_CONTROL_NOT_END;
            desc->driver_ref = job;
            desc->next_desc_virt = NULL;
            descriptor_set_phys(
                        &desc->next_desc_hi_phys,
                        &desc->next_desc_lo_phys,
                        0);

            // make the head or previous descriptor link to this one
            if (descriptor_no++ == 0) {
                job->descriptor = desc;
                job->descriptor_phys = desc_phys;
            } else {
                prev_desc->next_desc_virt = desc;
                descriptor_set_phys(
                            &prev_desc->next_desc_hi_phys,
                            &prev_desc->next_desc_lo_phys,
                            desc_phys);
            }
            dump_descriptor(desc);

            prev_desc = desc;
        }
    }
    prev_desc->control = ALTERA_DMA_DESC_CONTROL_END;

    // should be ready for hardware
    submit_transfer_to_hw_or_queue(adma, job);
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
long queue_data_transfer(struct altr_dma_dev* adma, struct minit_data_transfer_s* transfer,struct file* file)
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
    job->buffer = (char*)transfer->buffer;
    job->buffer_size = transfer->buffer_size;
    job->transfer_id = transfer->transfer_id;
    job->signal_number = transfer->signal_number;
    job->pid = transfer->pid;
    job->file = file;

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
    if (sg_alloc_table_from_pages(
        &job->sgt,
        pages,
        no_pages,
        pg_start_offset,
        job->buffer_size,
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
    rc = create_descriptor_list(adma, job, nents);
    if (rc) {
        goto err_unmap;
    }

    dump_job(job);
    // success
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

    VPRINTK("get_completed_data_transfers max %d transfers\n", max_elem);

    for (i = 0 ; i < max_elem ; ++i) {
        /// @todo need to only pop job if pid matches
        struct transfer_job_s* job = pop_job(&adma->transfers_done, &adma->done_lock);
        if (!job) {
            break;
        }

        VPRINTK("finished job at %p id %d status %x", job, job->transfer_id, job->status);

        statuses[i].transfer_id = job->transfer_id;
        statuses[i].status = job->status;

        // transfer probably done, data is in memory, status recorded, can now
        // destroy the job object
        kfree(job);
    }

    VPRINTK("found %d finished transfers\n", i);

    // 'i' should now be the number of jobs or max_elem
    return i;
}

/**
 * @brief returns the first job it finds with an id matching the perameter
 * @param adma
 * @param transfer_id
 * @return matching job or null
 *
 * searches through the various places that jobs can reside based on the order
 * that a job can move through them.
 */
static struct transfer_job_s* find_job_by_id(struct altr_dma_dev* adma, u32 transfer_id)
{
    struct transfer_job_s* job;
    unsigned long flags;

    spin_lock_irqsave(&adma->hardware_lock, flags);
    list_for_each_entry(job, &adma->transfers_ready_for_hardware, list) {
        if (job->transfer_id == transfer_id) {
            list_del(&job->list);
            spin_unlock_irqrestore(&adma->hardware_lock, flags);
            return job;
        }
    }
    if (adma->transfer_on_hardware &&
        adma->transfer_on_hardware->transfer_id == transfer_id)
    {
        job = adma->transfer_on_hardware;
        reset_dma_hardware(adma);
        adma->transfer_on_hardware = NULL; // it's not on hardware now!
        // restart the hardware whilst we have the lock
        start_transfer_unlocked(adma);

        spin_unlock_irqrestore(&adma->hardware_lock, flags);
        return job;
    }
    list_for_each_entry(job, &adma->post_hardware, list) {
        if (job->transfer_id == transfer_id) {
            list_del(&job->list);
            spin_unlock_irqrestore(&adma->hardware_lock, flags);
            return job;
        }
    }
    spin_unlock_irqrestore(&adma->hardware_lock,flags);

    return NULL;
}

/**
 * @brief returns the first job it finds with a file reference matching the perameter
 * @return matching job or null
 *
 * searches through the various places that jobs can reside based on the order
 * that a job can move through them.
 */
static struct transfer_job_s* find_job_by_file(struct altr_dma_dev* adma, struct file* file)
{
    struct transfer_job_s* job;
    unsigned long flags;

    spin_lock_irqsave(&adma->hardware_lock, flags);
    list_for_each_entry(job, &adma->transfers_ready_for_hardware, list) {
        DPRINTK("investigating job %p",job);
        dump_job(job);
        if (job->file == file) {
            DPRINTK("job found queued ready for hardware\n");
            list_del(&job->list);
            spin_unlock_irqrestore(&adma->hardware_lock, flags);
            return job;
        }
    }
    if (adma->transfer_on_hardware &&
        adma->transfer_on_hardware->file == file)
    {
        job = adma->transfer_on_hardware;
        DPRINTK("job found on hardware %p",job);
        dump_job(job);
        reset_dma_hardware(adma);
        adma->transfer_on_hardware = NULL; // it's not on hardware now!

        // the above search should have removed any queued jobs and we've just
        // ended the active job, restart the hardware knowing that we're not
        // going to start a job that we're going to want to immediately destroy
        start_transfer_unlocked(adma);

        spin_unlock_irqrestore(&adma->hardware_lock, flags);
        return job;
    }
    list_for_each_entry(job, &adma->post_hardware, list) {
        DPRINTK("investigating job %p",job);
        dump_job(job);
        if (job->file == file) {
            DPRINTK("job found queued post hardware\n");
            list_del(&job->list);
            spin_unlock_irqrestore(&adma->hardware_lock, flags);
            return job;
        }
    }
    spin_unlock_irqrestore(&adma->hardware_lock,flags);

    return NULL;
}


/**
 * @brief cancel data transfer with matching transfer_id
 * @param adma
 * @param transfer_id
 * @return 0 if found or -EINVAL if not found
 *
 * proceed through the queues looking for the transfer
 */
long cancel_data_transfer(struct altr_dma_dev* adma, u32 transfer_id)
{
    unsigned long flags;
    struct transfer_job_s* job;

    job = find_job_by_id(adma, transfer_id);
    if (job) {
        // free descriptors and unmap dma
        free_descriptor_list(adma, job);
        pci_unmap_sg(adma->pci_device, job->sgt.sgl, job->sgt.nents, DMA_FROM_DEVICE);
        kfree(job);
        return 0;
    }

    // may have already completed, but the userspace app still wants to cancel
    spin_lock_irqsave(&adma->done_lock, flags);
    list_for_each_entry(job, &adma->transfers_done, list) {
        if (job->transfer_id == transfer_id) {
            list_del(&job->list);
            kfree(job);
            spin_unlock_irqrestore(&adma->done_lock, flags);
            return 0;
        }
    }
    spin_unlock_irqrestore(&adma->done_lock, flags);

    // not found
    return -EINVAL;
}

/**
 * @brief cancel all the data transfers referencing a file
 */
void cancel_data_transfers_for_file(struct altr_dma_dev* adma, struct file* file)
{
    unsigned long flags;
    struct transfer_job_s* job;

    DPRINTK("cancel_data_transfer_for_file %p\n",file);
    do {
        job = find_job_by_file(adma, file);
        if (job) {
            DPRINTK("freeing job %p",job);
            dump_job(job);
            // free descriptors and unmap dma
            free_descriptor_list(adma, job);
            pci_unmap_sg(adma->pci_device, job->sgt.sgl, job->sgt.nents, DMA_FROM_DEVICE);
            kfree(job);
        }
    } while (job);

    // may have already completed, but still cancel
    spin_lock_irqsave(&adma->done_lock, flags);
    list_for_each_entry(job, &adma->transfers_done, list) {
        DPRINTK("investigating job %p",job);
        dump_job(job);
        if (job->file == file) {
            DPRINTK("freeing job %p",job);
            list_del(&job->list);
            kfree(job);
        }
    }
    spin_unlock_irqrestore(&adma->done_lock, flags);
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
    if (READL(adma->prefetcher_base + PRE_STATUS) & PRE_STATUS_IRQ) {
        WRITEL(PRE_STATUS_IRQ, adma->prefetcher_base + PRE_STATUS);
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

    VPRINTK("dma_isr\n");

    dump_dma_registers(adma);

    spin_lock(&adma->hardware_lock);
    // is there a transfer complete
    while (!dma_busy(adma) && adma->transfer_on_hardware) {
        u32 status;
        struct transfer_job_s* job = adma->transfer_on_hardware;
        adma->transfer_on_hardware = NULL;

        // check for error, reset core ?
        status = READL(adma->msgdma_base + MSGDMA_STATUS);
        job->status = status;
        if (status & (MSGDMA_STATUS_STOP_EARLY| MSGDMA_STATUS_STOP_ERROR|MSGDMA_STATUS_STOPPED)) {
            crazy_dump_debug(adma);
            reset_dma_hardware(adma);
        }

        list_add_tail( &job->list, &adma->post_hardware);

        // next waiting transfer
        start_transfer_unlocked(adma);

        // start the workqueue to inform the user
        queue_work(adma->finishing_queue, &adma->finishing_work);
    }

    spin_unlock(&adma->hardware_lock);
    return IRQ_HANDLED;
}

/**
 * @brief send a signal to a user-space process
 * @param signal_number
 * @param pid
 *
 * @todo check to see if adding the ability to send any signal to any process
 * constitutes a security problem.
 */
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

    VPRINTK("post_transfer\n");

    while ((job = pop_job(&adma->post_hardware, &adma->hardware_lock)) != NULL) {
        VPRINTK("post_transfer job %p\n",job);
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
    adma->max_transfer_size = get_max_transfer_size(adma);

    mdev->dma_isr_quick = dma_isr_quick;
    mdev->dma_isr = dma_isr;
    mdev->dma_dev = adma;
    INIT_LIST_HEAD(&adma->transfers_ready_for_hardware);
    INIT_LIST_HEAD(&adma->post_hardware);
    INIT_LIST_HEAD(&adma->transfers_done);

    // allocate a bunch of descriptors in coherant memory
    adma->descriptor_pool = dmam_pool_create(
                "MinION DMA",
                &mdev->pci_device->dev,
                sizeof(minit_dma_extdesc_t) * INITIAL_DESCRIPTOR_POOL_SIZE,
                4, // alignment
                0);// boundary
    if (!adma->descriptor_pool) {
        DPRINTK("Unable to allocate a pool of memory for dma descriptors\n");
        return -ENOMEM;
    }

    // create workqueue and work to do the post-transfer unmapping and callbacks
    adma->finishing_queue = create_singlethread_workqueue("minit-post-transfer");
    INIT_WORK(&adma->finishing_work, post_transfer);

    crazy_dump_debug(adma);
    return 0;
}

void altera_sgdma_remove(struct minit_device_s* mdev) {
    struct altr_dma_dev* adma = mdev->dma_dev;
    if (!adma) {
        return;
    }
    destroy_workqueue(adma->finishing_queue);

    mdev->dma_isr_quick = NULL;
    mdev->dma_isr = NULL;
    mdev->dma_dev = NULL;
    kfree(adma);
}
