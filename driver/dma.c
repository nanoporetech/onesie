/**
 * dma.c
 *
 * Copyright (C) 2019 Oxford Nanopore Technologies Ltd.
 *
 * Author: Richard Crewe <richard.crewe@nanoporetech.com>
 *
 * Hardware and memory interaction to do scatter-gather DMA
 *
 */
#include <linux/types.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include "ont_minit1c.h"
#include "ont_minit1c_reg.h"

/**
 * @brief extended descriptor with some software bits on the end
 */
struct __attribute__((__packed__, aligned(4) )) minit_dma_extdesc_s {
    u32 read_lo_phys;
    u32 write_lo_phys;
    u32 length;
    u32 next_desc_lo_phys;
    u32 bytes_transferred;
    u32 status;
    u32 reserved_0x18;
    u32 burst_sequence;
    u32 stride;
    u32 read_hi_phys;
    u32 write_hi_phys;
    u32 reserved_0x30;
    u32 reserved_0x34;
    u32 reserved_0x38;
    u32 control;
    void* driver_ref;
    struct minit_dma_extdesc_s* next_desc_virt;
};

typedef struct __attribute__((__packed__, aligned(4) )) minit_dma_extdesc_s minit_dma_extdesc_t;

/** Prefetcher registers */
#define PRE_CONTROL         0x00
#define PRE_NEXT_DESC_LO    0x04
#define PRE_NEXT_DESC_HI    0x08
#define PRE_DESC_POLL_FREQ  0x0c
#define PRE_STATUS          0x10

#define PRE_CTRL_PARK_MODE          (1 << 4)
#define PRE_CTRL_IRQ_MASK           (1 << 3)
#define PRE_CTRL_RESET              (1 << 2)
#define PRE_CTRL_POLL_ENABLE        (1 << 1)
#define PRE_CTRL_RUN                (1 << 0)

#define PRE_POLL_FREQ_MASK          0x0000ffff
#define PRE_POLL_FREQ_SHIFT         0

#define PRE_STATUS_IRQ              (1 << 0)

/** mSGDMA core registers */
#define MSGDMA_STATUS       0x00
#define MSGDMA_CONTROL      0x04
#define MSGDMA_RW_LEVEL     0x08
#define MSGDMA_RES_LEVEL    0x0c
#define MSGDMA_SEQ_NO       0x10
#define MSGDMA_CONFIG_1     0x14
#define MSGDMA_CONFIG_2     0x18
#define MSGDMA_VERSION      0x1c

#define MSGDMA_STATUS_IRQ           (1 << 9)
#define MSGDMA_STATUS_STOP_EARLY    (1 << 8)
#define MSGDMA_STATUS_STOP_ERROR    (1 << 7)
#define MSGDMA_STATUS_RESETTING     (1 << 6)
#define MSGDMA_STATUS_STOPPED       (1 << 5)
#define MSGDMA_STATUS_RESP_FULL     (1 << 4)
#define MSGDMA_STATUS_RESP_EMPTY    (1 << 3)
#define MSGDMA_STATUS_DESC_FULL     (1 << 2)
#define MSGDMA_STATUS_DESC_EMPTY    (1 << 1)
#define MSGDMA_STATUS_BUSY          (1 << 0)

#define MSGDMA_CTRL_STOP_DESC       (1 << 5)
#define MSGDMA_CTRL_IRQ_MASK        (1 << 4)
#define MSGDMA_CTRL_STOP_EARLY      (1 << 3)
#define MSGDMA_CTRL_STOP_ERROR      (1 << 2)
#define MSGDMA_CTRL_RESET_DISPATCH  (1 << 1)
#define MSGDMA_CTRL_STOP_DISPATCH   (1 << 0)

#define MSGDMA_WRITE_LEVEL_MASK     0xffff0000
#define MSGDMA_WRITE_LEVEL_SHIFT    16
#define MSGDMA_READ_LEVEL_MASK      0x0000ffff
#define MSGDMA_READ_LEVEL_SHIFT     0

#define MSGDMA_RESP_LEVEL_MASK      0x0000ffff
#define MSGDMA_RESP_LEVEL_SHIFT     0

#define MSGDMA_WRITE_SEQ_MASK       0xffff0000
#define MSGDMA_WRITE_SEQ_SHIFT      16
#define MSGDMA_READ_SEQ_MASK        0x0000ffff
#define MSGDMA_READ_SEQ_SHIFT       0

#define MSGDMA_TYPE_MASK            0x0000ff00
#define MSGDMA_TYPE_SHIFT           8
#define MSGDMA_VERSION_MASK         0x000000ff
#define MSGDMA_VERSION_SHIFT        0

struct altr_dma_dev {
    void __iomem* msgdma_base;
    void __iomem* prefetcher_base;
    // the chain of descriptors that the hardware is currently working on
    minit_dma_extdesc_t* active_descriptor_chain;


    minit_dma_extdesc_t* transfer_queue;
};

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
    printk(KERN_ERR"PRE_CONTROL = %0x08x\n",reg);
    printk(KERN_ERR" %s %s %s %s %s\n",
           reg & (1 << 4) ? "PARK_MODE" : ".",
           reg & (1 << 3) ? "IRQ_EN" : ".",
           reg & (1 << 2) ? "RESET" : ".",
           reg & (1 << 1) ? "POLL_EN" : ".",
           reg & (1 << 0) ? "RUN" : ".");


    hi = reg = readl(adma->prefetcher_base + PRE_NEXT_DESC_LO);
    printk(KERN_ERR"PRE_NEXT_DESC_LO = %0x08x\n",reg);
    reg = readl(adma->prefetcher_base + PRE_NEXT_DESC_HI);
    printk(KERN_ERR"PRE_NEXT_DESC_HI = %0x08x\n",reg);
    printk(KERN_ERR" next descriptor address 0x%016llx\n", (hi << 32) + reg);

    reg = readl(adma->prefetcher_base + PRE_DESC_POLL_FREQ);
    printk(KERN_ERR"PRE_DESC_POLL_FREQ = %0x08x\n",reg);
    printk(KERN_ERR" poll frequency %d cycles\n",
           reg & 0x0000ffff);

    reg = readl(adma->prefetcher_base + PRE_STATUS);
    printk(KERN_ERR"PRE_STATUS = %0x08x\n",reg);
    printk(KERN_ERR" %s\n",
           (reg & 0x00000001) ? "IRQ": ".");

    /// @TODO dump descriptor chains.
    desc = adma->active_descriptor_chain;
    while (desc) {
        dump_descriptor(desc);
        desc = desc->next_desc_virt;
    }
}

static irqreturn_t dma_isr_quick(int irq_no, void* dev)
{
    return IRQ_HANDLED;
}

static irqreturn_t dma_isr(int irq_no, void* dev)
{
    return IRQ_HANDLED;
}



/*
 * ioctl -> create job structure
 * convert to scatterlist and map
 * queue for converting to descriptor list
 *
 * convert to descriptor list
 * queue for hardware
 *
 * submit to hardware
 *
 * irq from hardware, start bh
 *
 * in bh (start next job on hw queue) free descriptor list resources, send to cleanup queue
 *
 * from cleanup queue, unmap, signal user-space, add to done queue
 *
 */

long queue_data_transfer(struct minit_data_transfer_s* transfer)
{
    return -1;
}

u32 get_completed_data_transfers(u32 max_elem, struct minit_transfer_status* statuses)
{
    return 0;
}

long cancel_data_transfer(u32 transfer_no)
{
    return -1;
}


int altera_sgdma_probe(struct minit_device_s* mdev) {
    // confirm correct version of hardware

    // construct dma coordination structure and link with driver
    struct altr_dma_dev* adma = kzalloc(sizeof(struct altr_dma_dev), GFP_KERNEL);
    if (!adma) {
        return -ENOMEM;
    }
    adma->msgdma_base = mdev->ctrl_bar + ASIC_HS_DMA_BASE;
    adma->prefetcher_base = mdev->ctrl_bar + ASIC_HS_DMA_PREF_BASE;



    mdev->dma_isr_quick = dma_isr_quick;
    mdev->dma_isr = dma_isr;
    mdev->dma_dev = adma;


    crazy_dump_debug(adma);
    return -1;
}

void altera_sgdma_remove(struct minit_device_s* mdev) {

}
