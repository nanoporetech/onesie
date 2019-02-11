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
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/scatterlist.h>
#include "ont_minit1c.h"
/**
 * @brief extended descriptor with some software bits on the end
 */
struct __attribute__((__packed__, aligned(4) )) minit_dma_extdesc_s {
    u32 read_lo_phys;
    u32 write_lo_phys;
    u32 length_lo;
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
    struct minit_dma_desc_s* next_desc_virt;
};

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

struct altera_msgdma {
    void __iomem* msgdma_base;
    void __iomem* prefetcher_base;
};

/**
 * Dump registers and descriptor chains and anything else useful to the
 * kernel log
 *
 * @param adma driver structure
 */
static void crazy_dump_debug(struct altera_msgdma* adma) {
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

}


int altera_sgdma_probe(struct minit_device_s* mdev) {
    crazy_dump_debug(mdev);
    return -1;
}

void altera_sgdma_remove(struct minit_device_s* mdev) {

}
