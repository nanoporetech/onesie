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

#ifndef ONT_ALTERA_DMA_H
#define ONT_ALTERA_DMA_H

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

#define ALTERA_DMA_PRE_START (PRE_CTRL_IRQ_MASK|PRE_CTRL_RUN)

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


/// @TODO: correct controls for DMA with an interrupt on the last transfer or
/// an error
#define ALTERA_DMA_DESC_CONTROL_NOT_END 0x0
#define ALTERA_DMA_DESC_CONTROL_END     0x0

struct transfer_job_s;

/**
 * @brief extended descriptor with some software bits on the end
 */
struct __attribute__((__packed__, aligned(4) )) minit_dma_extdesc_s {
    u32 read_lo_phys;           // 00
    u32 write_lo_phys;          // 04
    u32 length;                 // 08
    u32 next_desc_lo_phys;      // 0c
    u32 bytes_transferred;      // 10
    u32 status;                 // 14
    u32 reserved_0x18;          // 18
    u32 burst_sequence;         // 1c
    u32 stride;                 // 20
    u32 read_hi_phys;           // 24
    u32 write_hi_phys;          // 28
    u32 next_desc_hi_phys;      // 2c
    u32 reserved_0x30;          // 30
    u32 reserved_0x34;          // 34
    u32 reserved_0x38;          // 38
    u32 control;                // 3c
    struct transfer_job_s* driver_ref;
    struct minit_dma_extdesc_s* next_desc_virt;
};

typedef struct __attribute__((__packed__, aligned(4) )) minit_dma_extdesc_s minit_dma_extdesc_t;

struct transfer_job_s {
    struct list_head list;
    char __user* buffer;
    u32 buffer_size;
    u32 transfer_id;
    int signal_number;
    int pid;
    struct sg_table sgt;
    u32 status;

    // DMA descriptor chain virtual and dma addreses
    minit_dma_extdesc_t* descriptor;
    dma_addr_t descriptor_phys;
};


struct altr_dma_dev {
    void __iomem* msgdma_base;
    void __iomem* prefetcher_base;
    struct pci_dev* pci_device;
    unsigned long max_transfer_size;

    // This covers transfers_ready_for_hardware, transfers_on_hardware and post_hardware
    spinlock_t hardware_lock;

    struct list_head transfers_ready_for_hardware;
    struct transfer_job_s* transfer_on_hardware;
    struct list_head post_hardware;

    // covers transfers_done, do-not attempt to take the hardware_lock after
    // taking this lock as that can deadlock.
    spinlock_t done_lock;
    struct list_head transfers_done;

    struct dma_pool* descriptor_pool;
    struct workqueue_struct* finishing_queue;
    struct work_struct finishing_work;
};
#endif // ONT_ALTERA_DMA_H
