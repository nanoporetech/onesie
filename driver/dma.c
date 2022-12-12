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
#include <linux/delay.h>

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
#include <linux/sched.h>
#else
#include <linux/sched/signal.h>
#endif

#include "minion_top.h"
#include "minion_ioctl.h"
#include "minion_reg.h"
#include "dma.h"


/**
 * Dump registers and anything else useful to the
 * kernel log
 *
 * @param adma driver structure
 */
static void dump_dma_registers(struct altr_dma_dev* adma) {
    u32 reg;
    u64 hi;

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
    printk(KERN_ERR"PRE_NEXT_DESC_HI = 0x%08x\n",(u32)hi);
    printk(KERN_ERR" next descriptor address 0x%016llx\n", (hi << 32) + reg);

    reg = readl(adma->prefetcher_base + PRE_DESC_POLL_FREQ);
    printk(KERN_ERR"PRE_DESC_POLL_FREQ = 0x%08x\n",reg);
    printk(KERN_ERR" poll frequency %d cycles\n",
           reg & 0x0000ffff);

    reg = readl(adma->prefetcher_base + PRE_STATUS);
    printk(KERN_ERR"PRE_STATUS = 0x%08x\n",reg);
    printk(KERN_ERR" %s\n",
           (reg & 0x00000001) ? "IRQ": ".");
}

/**
 * @brief dump the altr_dma_dev structure and its children
 */
void dma_dump_debug(struct altr_dma_dev* adma)
{
    if (!adma) {
        printk(KERN_ERR"struct altr_dma_dev in NULL!\n");
        return;
    }

    printk(KERN_ERR"msgdma base %p\n",adma->msgdma_base);
    printk(KERN_ERR"prefetcher base %p\n",adma->prefetcher_base);
    dump_dma_registers(adma);

    printk(KERN_ERR"pci_device %p\n",adma->pci_device);
    printk(KERN_ERR"max transfer size 0x%x (%d)\n",adma->max_transfer_size,adma->max_transfer_size);
    printk(KERN_ERR"hardware lock %s\n", spin_is_locked(&adma->hardware_lock) ? "locked" : "unlocked" );

    printk(KERN_ERR"transfers on hardware %s\n", (!list_empty(&adma->transfers_on_hardware) ? "Yes" : "No"));
    printk(KERN_ERR"post-hardware transfers %s\n", (!list_empty(&adma->post_hardware) ? "Yes" : "No") );

    printk(KERN_ERR"done lock %s\n", spin_is_locked(&adma->done_lock) ? "locked" : "unlocked");
    printk(KERN_ERR"done transfers %s\n", (!list_empty(&adma->transfers_done) ? "Yes" : "No") );
    printk(KERN_ERR"descriptor pool %p\n",adma->descriptor_pool);
    printk(KERN_ERR"finishing queue %p\n",adma->finishing_queue);
    printk(KERN_ERR"terminal descriptor %p (phys 0x%016llx)\n",
           adma->terminal_desc,adma->terminal_desc_phys);
}

/**
 * @brief extract the maximum transfer size from the hardware config registers
 * or use a sensible default if that register isn't implemented.
 * @param adma
 * @return maximum transfer-size in bytes
 */
static unsigned int get_max_transfer_size(struct altr_dma_dev* adma)
{
    u32 reg;
    reg = READL(adma->msgdma_base + MSGDMA_CONFIG_1);
    if (!reg) {
        return 2048;
    } else {
        // use the max transfer sizefrom the hardware config register
        return 1 << (((reg & 0xf8000000) >> 27) + 10);
    }
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
    rmb();
    return READL(adma->msgdma_base + MSGDMA_STATUS) & MSGDMA_STATUS_BUSY;
}

/**
 * @brief reset dma hardware. will sleep.
 * @param adma
 */
static void reset_dma_hardware(struct altr_dma_dev* adma)
{
    unsigned int attempt;
    static const unsigned long max_wait = HZ / 10;

    // There seems to be a circular dependency between the two DMA cores.
    // Resetting them both can sometimes take a couple of goes.
    for (attempt = 0; attempt < 2; ++attempt) {
        unsigned long time_out;
        bool prefetch_reset_ok = false;
        bool sgdma_reset_ok = false;

        // reset prefetcher, then wait a limited time for it to clear
        WRITEL(PRE_CTRL_RESET, adma->prefetcher_base + PRE_CONTROL);
        time_out = jiffies + max_wait;
        while(jiffies < time_out) {
            // is prefetcher reset complete
            if (!(READL(adma->prefetcher_base + PRE_CONTROL) & PRE_CTRL_RESET)) {
                prefetch_reset_ok = true;
                break;
            }
            msleep(1);
        }

        // reset dispatcher in msgdma core then wait a limited time for it to clear
        WRITEL(MSGDMA_CTRL_RESET_DISPATCH, adma->msgdma_base + MSGDMA_CONTROL);
        time_out = jiffies + max_wait;
        while(jiffies < time_out) {
            // is mSGDMA/dispatcher reset complete
            if (!(READL(adma->msgdma_base + MSGDMA_STATUS) & MSGDMA_STATUS_RESETTING)) {
                sgdma_reset_ok = true;
                break;
            }
            msleep(1);
        }
        if (prefetch_reset_ok && sgdma_reset_ok) {
            return;
        }
    }
}

// copies the descriptors, but makes sure to copy the control field last.
static void desc_copy(minion_dma_extdesc_t* dst, const minion_dma_extdesc_t* src)
{
    dst->read_lo_phys  = src->read_lo_phys;
    dst->write_lo_phys = src->write_lo_phys;
    dst->length = src->length;
    dst->next_desc_lo_phys = src->next_desc_lo_phys;
    dst->bytes_transferred = src->bytes_transferred;
    dst->status = src->status;
    dst->reserved_0x18 = 0;
    dst->burst_sequence = src->burst_sequence;
    dst->stride = src->stride;
    dst->read_hi_phys = src->read_hi_phys;
    dst->write_hi_phys = src->write_hi_phys;
    dst->next_desc_hi_phys = src->next_desc_hi_phys;
    dst->reserved_0x30 = 0;
    dst->reserved_0x34 = 0;
    dst->reserved_0x38 = 0;
    dst->driver_ref = src->driver_ref;
    dst->next_desc_virt = src->next_desc_virt;
    wmb();

    // writing this can cause hardware watching the memory to spring into life
    // so barriers before and after to make sure things are written to physical
    // memory
    dst->control = src->control;
    wmb();
}

static void start_transfer_idle_nolocking(struct altr_dma_dev* adma, struct transfer_job_s* job)
{
    VPRINTK("start from idle\n");
    // stop prefetcher core
    WRITEL(0, adma->prefetcher_base + PRE_CONTROL);
    wmb();

    // free terminal descriptor
    dma_pool_free(adma->descriptor_pool, adma->terminal_desc, adma->terminal_desc_phys);

    // register our terminal descriptor
    adma->terminal_desc = job->terminal_desc;
    adma->terminal_desc_phys = job->terminal_desc_phys;
    wmb();

    // write address of first descrptor into prefetcher
    WRITEL((u32)(job->descriptor_phys >> 32), adma->prefetcher_base + PRE_NEXT_DESC_HI);
    WRITEL((u32)(job->descriptor_phys & 0x00000000ffffffff), adma->prefetcher_base + PRE_NEXT_DESC_LO);
    wmb();

    // set descriptor polling interval, 2048 chosen as is just less than the
    //62.5MHz / max sample-rate of 30 kHz
    WRITEL(2048, adma->prefetcher_base + PRE_DESC_POLL_FREQ);

    // add this job to the end of the list of jobs on hardware
    list_add_tail(&job->list, &adma->transfers_on_hardware);

    // start the prefetcher core
    WRITEL(ALTERA_DMA_PRE_START, adma->prefetcher_base + PRE_CONTROL);
    wmb();
    udelay(10);
}

// if busy, then:
static void add_transfer_busy_nolocking(struct altr_dma_dev* adma, struct transfer_job_s* job)
{
    minion_dma_extdesc_t* desc;
    minion_dma_extdesc_t* first;
    dma_addr_t desc_phys;
    dma_addr_t first_phys;
    VPRINTK("add to end of descriptor chain\n");
    // find the terminal descriptor for the executing dma chain
    desc = adma->terminal_desc;
    desc_phys = adma->terminal_desc_phys;

    // record the job's first descriptor as we'll be freeing it later
    first = job->descriptor;
    first_phys = job->descriptor_phys;

    // replace the driver's ref to terminal with terminal from this job
    adma->terminal_desc = job->terminal_desc;
    adma->terminal_desc_phys = job->terminal_desc_phys;

    // copy first descriptor's data into terminal leaving the control until
    // last, this connects the descriptor chains.
    desc_copy(desc, job->descriptor);

    job->descriptor = desc;
    job->descriptor_phys = desc_phys;

    // add this job to the end of the list of jobs on hardware
    list_add_tail(&job->list, &adma->transfers_on_hardware);

    // free first descriptor.
    dma_pool_free(adma->descriptor_pool, first, first_phys);
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

    // increment sequence_no, avoiding zero
    if (++adma->sequence_no == 0) {
        ++adma->sequence_no;
    }
    job->sequence_no = adma->sequence_no;
    if (likely(dma_busy(adma))) {
        add_transfer_busy_nolocking(adma,job);
    } else {
        start_transfer_idle_nolocking(adma,job);
    }
    // unlock something
    spin_unlock_irqrestore(&adma->hardware_lock, flags);
}

/**
 * @brief walk descriptor list freeing entries and returning resources to descritor pool
 * @param adma
 * @param job
 *
 * walk descriptor list freeing entries and returning resources to descritor
 * pool, as we're likely to be chained to following desciptors, have to be
 * careful not to free their descriptors. Do this by checking sequence-number.
 */
static void free_descriptor_list(struct altr_dma_dev* adma, struct transfer_job_s* job)
{
    minion_dma_extdesc_t* desc = job->descriptor;
    dma_addr_t desc_phys = job->descriptor_phys;
    VPRINTK("free desc list\n");
    while (desc && desc->driver_ref == job) {
        minion_dma_extdesc_t* next = desc->next_desc_virt;
        dma_addr_t next_phys = descriptor_get_phys(&desc->next_desc_hi_phys, &desc->next_desc_lo_phys);
        dma_pool_free(adma->descriptor_pool, desc, desc_phys);
        desc = next;
        desc_phys = next_phys;
    }
    job->descriptor = NULL;
    job->descriptor_phys = 0;
}

/**
 * @brief Allocate, initialise and link-in a terminal descriptor
 * @param adma
 * @param job
 * @param prev
 * @return
 */
static long terminal_descriptor(struct altr_dma_dev* adma, struct transfer_job_s* job, minion_dma_extdesc_t* prev)
{
    minion_dma_extdesc_t* desc;
    dma_addr_t desc_phys;
    desc = dma_pool_alloc(adma->descriptor_pool, GFP_KERNEL, &desc_phys);
    if (!desc) {
        DPRINTK("error, couldn't allocate dma descriptor\n");
        return -ENOMEM;
    }
    VPRINTK("terminal descriptor allocated at %p\n",desc);
    memset(desc, 0, sizeof(minion_dma_extdesc_t));
    job->terminal_desc = desc;
    job->terminal_desc_phys = desc_phys;
    prev->next_desc_virt = desc;
    descriptor_set_phys(
                &prev->next_desc_hi_phys,
                &prev->next_desc_lo_phys,
                desc_phys);
    return 0;
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

    minion_dma_extdesc_t* prev_desc = NULL;

    VPRINTK("create_descriptor_list\n");
    // allocate DMA descriptors for transfer from dma-pool
    for(sg_index = 0, sg_elem = job->sgt.sgl; sg_index < nents; sg_index++, sg_elem++) {
        unsigned int length = sg_dma_len(sg_elem);
        dma_addr_t dma_address = sg_dma_address(sg_elem);
        while (length) {
            minion_dma_extdesc_t* desc;
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
            desc->burst_sequence = job->sequence_no; // burst left at 0
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

            prev_desc = desc;
        }
    }
    prev_desc->control = ALTERA_DMA_DESC_CONTROL_END;
    rc = terminal_descriptor(adma, job, prev_desc);
    if (rc < 0) {
        goto err_free_descriptors;
    }
    return 0;

err_free_descriptors:
    free_descriptor_list(adma, job);

    return rc;
}

/**
 * Map in the user pages in `buffer` and construct a scatter list out of them for `job`.
 */
static int setup_dma(struct altr_dma_dev* adma, struct transfer_job_s* job, char __user* buffer, u32 buffer_size)
{
    unsigned long pg_start, pg_start_offset, pg_end;
    unsigned int page_count_for_buffer = 0;
    struct page** pages = NULL;
    int loaded_page_count = 0;
    int nents;
    int rc;
    int i;

    if (buffer_size == 0) {
        // there's nothing to load...
        return -EINVAL;
    }

    // convert to scaterlist and do dma mapping
    pg_start = (unsigned long)buffer & PAGE_MASK;
    pg_start_offset = (unsigned long)buffer & ~PAGE_MASK;
    pg_end = PAGE_ALIGN((unsigned long)(buffer + buffer_size));
    page_count_for_buffer = (pg_end - pg_start) >> PAGE_SHIFT;

    // Allocate memory for page list
    pages = kzalloc(sizeof(struct page*) * page_count_for_buffer, GFP_KERNEL);
    if (!pages) {
        adma_dbg(adma, "Couldn't allocate memory for page-table\n");
        return -ENOMEM;
    }

    // Pull the user buffer into kernel memory space
    loaded_page_count = get_user_pages_fast(pg_start, page_count_for_buffer, 1, pages);
    if (loaded_page_count != page_count_for_buffer) {
        adma_dbg(adma, "Failed to map in user pages\n");
        goto err;
    }
    VPRINTK("start page addr %lx, end page addr %lx, start offset %lx\n",
        pg_start, pg_end, pg_start_offset);

    // create scatterlist
    if (sg_alloc_table_from_pages(
        &job->sgt,
        pages,
        loaded_page_count,
        pg_start_offset,
        buffer_size,
        GFP_KERNEL))
    {
        adma_dbg(adma, "Failed to allocate memory for scatterlist\n");
        goto err;
    }

    nents = dma_map_sg(&adma->pci_device->dev, job->sgt.sgl, job->sgt.nents, DMA_FROM_DEVICE);
    if (!nents) {
        adma_dbg(adma, "Error mapping dma to bus addresses\n");
        goto err_scatterlist_created;
    }

    rc = create_descriptor_list(adma, job, nents);
    if (rc) {
        goto err_mapped;
    }

    VPRINTK("Mapped %d DMA transfers for userspace buffer %p (len %d) using %d pages at %p\n",
        nents,
        job->buffer,
        job->buffer_size,
        loaded_page_count,
        pages);

    job->buffer = buffer;
    job->buffer_size = buffer_size;
    job->pages = pages;
    job->no_pages = loaded_page_count;
    return 0;

err_mapped:
    dma_unmap_sg(&adma->pci_device->dev, job->sgt.sgl, job->sgt.nents, DMA_FROM_DEVICE);
err_scatterlist_created:
    sg_free_table(&job->sgt);
err:
    for (i = 0; i < loaded_page_count; ++i) {
        put_page(pages[i]);
    }
    kfree(pages);
    return -ENOMEM;
}

static void mark_buffer_changed(struct transfer_job_s* job)
{
    unsigned i;
    for (i = 0; i < job->no_pages; ++i) {
        SetPageDirty(job->pages[i]);
    }
}

/**
 * Release buffer pages and release memory used by the scatterlist.
 */
static void teardown_dma(struct altr_dma_dev* adma, struct transfer_job_s* job)
{
    unsigned i;

    if (job->no_pages == 0) {
        // calling discard twice? or didn't build it in the first place?
        BUG();
        return;
    }

    free_descriptor_list(adma, job);
    dma_unmap_sg(&adma->pci_device->dev, job->sgt.sgl, job->sgt.nents, DMA_FROM_DEVICE);

    sg_free_table(&job->sgt);
    for (i = 0; i < job->no_pages; ++i) {
        put_page(job->pages[i]);
    }
    kfree(job->pages);
    job->pages = NULL;
    job->no_pages = 0;
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
long queue_data_transfer(struct altr_dma_dev* adma, struct minion_data_transfer_s* transfer,struct file* file)
{
    long rc = 0;
    struct transfer_job_s* job = NULL;

    // check the buffer the user supplied has the correct alignment to avoid
    // cache-coherency issues
    if (transfer->buffer & (ARCH_DMA_MINALIGN-1)) {
        printk(KERN_ERR"Start of DMA buffer not aligned on %d-byte boundary\n",ARCH_DMA_MINALIGN);
        return -EINVAL;
    }

    if ((transfer->buffer + transfer->buffer_size) & (ARCH_DMA_MINALIGN-1)) {
        printk(KERN_ERR"End of DMA buffer not aligned on %d-byte boundary\n",ARCH_DMA_MINALIGN);
        return -EINVAL;
    }

    // create job structure, freed in get_completed_data_transfers, or cancel_data_transfer
    job = alloc_transfer_job();
    if (!job) {
        DPRINTK("Failed to allocate memory for transfer_job\n");
        return -ENOMEM;
    }
    job->transfer_id = transfer->transfer_id;
    job->signal_number = transfer->signal_number;
    job->pid = find_get_pid(task_pid_nr(current));
    job->file = file;

    rc = setup_dma(adma, job, (char*)transfer->buffer, transfer->buffer_size);
    if (rc != 0) {
        free_transfer_job(job);
        return rc;
    }

    submit_transfer_to_hw_or_queue(adma, job);
    return 0;
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
u32 get_completed_data_transfers(struct altr_dma_dev* adma, u32 max_elem, struct minion_transfer_status_s* statuses)
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
        free_transfer_job(job);
    }

    VPRINTK("found %d finished transfers\n", i);

    // 'i' should now be the number of jobs or max_elem
    return i;
}

/**
 * @brief cancel all data transfers
 * @param adma
 * @return 0 if found or -EINVAL if not found
 *
 * stops the hardware and cancels and disposes of all transfers, both pending
 * and complete.
 */
long cancel_data_transfers(struct altr_dma_dev* adma)
{
    unsigned long flags;
    struct transfer_job_s* job;
    struct transfer_job_s* temp;

    // stop hardware
    reset_dma_hardware(adma);
    spin_lock_irqsave(&adma->hardware_lock, flags);

    // dispose of all the jobs either on the hardware or associated with the descriptor chain
    list_for_each_entry_safe(job, temp, &adma->transfers_on_hardware, list) {
        DPRINTK("investigating job on hw %p\n",job);

        if (job) {
            // free descriptors and unmap dma, release pages
            list_del(&job->list);
            teardown_dma(adma, job);
            free_transfer_job(job);
        }
    }
    list_for_each_entry_safe(job, temp, &adma->post_hardware, list) {
        DPRINTK("investigating job post hw %p\n",job);

        if (job) {
            // free descriptors and unmap dma, release pages
            list_del(&job->list);
            mark_buffer_changed(job);
            teardown_dma(adma, job);
            free_transfer_job(job);
        }
    }
    spin_unlock_irqrestore(&adma->hardware_lock, flags);

    // may have already completed, but the userspace app still wants to cancel
    spin_lock_irqsave(&adma->done_lock, flags);
    list_for_each_entry_safe(job, temp, &adma->transfers_done, list) {
        DPRINTK("investigating job complete %p\n",job);
        if (job) {
            list_del(&job->list);
            free_transfer_job(job);
        }
    }
    spin_unlock_irqrestore(&adma->done_lock, flags);

    // not found
    return 0;
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
    struct transfer_job_s* job;
    struct transfer_job_s* temp;
    struct altr_dma_dev* adma = (struct altr_dma_dev*)dev;
    if (!adma) {
        // uh oh!
        DPRINTK("Bottom half, called without device sturcture\n");
        return IRQ_NONE;
    }

    VPRINTK("dma_isr\n");

    // repeat this until no new transfers are found
    spin_lock(&adma->hardware_lock);

    // find out which jobs are still running and which are done, by checking
    // that their descriptors are owened by sw and have bytes transferred match
    // length and > 0.
    list_for_each_entry_safe(job, temp, &adma->transfers_on_hardware, list) {
        u32 transferred_bytes = 0;
        minion_dma_extdesc_t* desc;

        VPRINTK("Job %p\n", job);

        desc = job->descriptor;
        // iterate through this job's descriptors
        while(desc && desc->driver_ref == job) {
            // is it owned by sw?
            if (desc->control & ALTERA_DMA_DESC_CONTROL_HW_OWNED) {
                break;
            }

            transferred_bytes += desc->length;
            desc = desc->next_desc_virt;
        }

        // if job hasn't transferred all data then we've reached the end of the
        // complete jobs, stop processing this list
        VPRINTK("transferred %d, expect %d\n",transferred_bytes, job->buffer_size);
        if (transferred_bytes < job->buffer_size) {
            break;
        }

        // start the workqueue to do the post hardware cleanup and inform user it's done
        list_move_tail(&job->list, &adma->post_hardware);
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
static void send_signal(const int signal_number, struct pid *pid)
{
    if (signal_number && pid) {
        int rc;
        struct task_struct* task;

        rcu_read_lock();
        task = pid_task(pid, PIDTYPE_PID);
        rcu_read_unlock();
        if (task) {
            rc = send_sig(signal_number, task, 0);
            if (rc < 0) {
                DPRINTK("error sending signal\n");
            }
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
        mark_buffer_changed(job);
        teardown_dma(adma, job);

        // move to done
        push_job(&adma->transfers_done, &adma->done_lock, job);
        send_signal(job->signal_number, job->pid);
    }
}

int altera_sgdma_probe(struct minion_device_s* mdev) {
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
    INIT_LIST_HEAD(&adma->transfers_on_hardware);
    INIT_LIST_HEAD(&adma->post_hardware);
    INIT_LIST_HEAD(&adma->transfers_done);

    // allocate a bunch of descriptors in coherant memory
    adma->descriptor_pool = dmam_pool_create(
                "MinION DMA",
                &mdev->pci_device->dev,
                sizeof(minion_dma_extdesc_t),
                4, // alignment
                256);// boundary
    if (!adma->descriptor_pool) {
        DPRINTK("Unable to allocate a pool of memory for dma descriptors\n");
        return -ENOMEM;
    }

    // create workqueue and work to do the post-transfer unmapping and callbacks
    adma->finishing_queue = create_singlethread_workqueue("minion-post-transfer");
    INIT_WORK(&adma->finishing_work, post_transfer);

    // allocate a terminal descriptor
    adma->terminal_desc = dma_pool_alloc(adma->descriptor_pool, GFP_KERNEL, &adma->terminal_desc_phys);
    if (!adma->terminal_desc) {
        DPRINTK("Unable to allocate a terminal dma descriptors\n");
        return -ENOMEM;
    }
    memset(adma->terminal_desc, 0, sizeof(minion_dma_extdesc_t));

    return 0;
}

void altera_sgdma_remove(void* ptr) {
    struct minion_device_s* mdev = (struct minion_device_s* )ptr;
    struct altr_dma_dev* adma = mdev->dma_dev;
    DPRINTK("altera_sgdma_remove\n");
    if (!adma) {
        return;
    }

    if (adma->terminal_desc) {
        dma_pool_free(adma->descriptor_pool, adma->terminal_desc, adma->terminal_desc_phys);
        adma->terminal_desc = NULL;
        adma->terminal_desc_phys = 0;
    }

    destroy_workqueue(adma->finishing_queue);

    mdev->dma_isr_quick = NULL;
    mdev->dma_isr = NULL;
    mdev->dma_dev = NULL;
    kfree(adma);
}
