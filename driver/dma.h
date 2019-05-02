/**
 * dma.h
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

#define ALTERA_DMA_PRE_START (PRE_CTRL_IRQ_MASK|PRE_CTRL_RUN|PRE_CTRL_POLL_ENABLE)

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



struct transfer_job_s;

/**
 * @brief extended descriptor with some software bits on the end
 */
struct __attribute__((__packed__, aligned(4) )) minion_dma_extdesc_s {
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
    struct minion_dma_extdesc_s* next_desc_virt;
};

typedef struct __attribute__((__packed__, aligned(4) )) minion_dma_extdesc_s minion_dma_extdesc_t;

#define ALTERA_DMA_DESC_CONTROL_TX_CHAN_SHIFT       0
#define ALTERA_DMA_DESC_CONTROL_TX_CHAN_MASK        0x000000ff
#define ALTERA_DMA_DESC_CONTROL_SOP                 (1 <<  8)
#define ALTERA_DMA_DESC_CONTROL_EOP                 (1 <<  9)
#define ALTERA_DMA_DESC_CONTROL_PARK_RD             (1 << 10)
#define ALTERA_DMA_DESC_CONTROL_PARK_WT             (1 << 11)
#define ALTERA_DMA_DESC_CONTROL_END_ON_EOP          (1 << 12)
#define ALTERA_DMA_DESC_CONTROL_TX_IRQ              (1 << 14)
#define ALTERA_DMA_DESC_CONTROL_EARLY_IRQ           (1 << 15)
#define ALTERA_DMA_DESC_CONTROL_ERROR_IRQ_SHIFT     16
#define ALTERA_DMA_DESC_CONTROL_ERROR_IRQ_MASK      0x00ff0000
#define ALTERA_DMA_DESC_CONTROL_EARLY               (1 << 24)
#define ALTERA_DMA_DESC_CONTROL_HW_OWNED            (1 << 30)
#define ALTERA_DMA_DESC_CONTROL_GO                  (1 << 31)

#define ALTERA_DMA_DESC_CONTROL_NOT_END (ALTERA_DMA_DESC_CONTROL_EARLY |\
                                         ALTERA_DMA_DESC_CONTROL_HW_OWNED |\
                                         ALTERA_DMA_DESC_CONTROL_GO)

#define ALTERA_DMA_DESC_CONTROL_END     (ALTERA_DMA_DESC_CONTROL_EARLY |\
                                         ALTERA_DMA_DESC_CONTROL_TX_IRQ |\
                                         ALTERA_DMA_DESC_CONTROL_HW_OWNED |\
                                         ALTERA_DMA_DESC_CONTROL_GO)

struct transfer_job_s {
    struct list_head list;
    char __user* buffer;
    u32 buffer_size;
    u32 transfer_id; // for the user to track transfers
    int signal_number;
    int pid;
    struct sg_table sgt;
    u32 status;
    struct file* file;

    // DMA descriptor chain virtual and dma addreses
    minion_dma_extdesc_t* descriptor;
    dma_addr_t descriptor_phys;

    // DMA descriptor terminating the descriptor chain
    minion_dma_extdesc_t* terminal_desc;
    dma_addr_t terminal_desc_phys;

    u16 sequence_no; // for us to track transfers.

    struct page** pages;
    unsigned int no_pages;
};


struct altr_dma_dev {
    void __iomem* msgdma_base;
    void __iomem* prefetcher_base;
    struct pci_dev* pci_device;
    unsigned int max_transfer_size;

    // This covers transfers_ready_for_hardware, transfers_on_hardware and post_hardware
    spinlock_t hardware_lock;

    struct list_head transfers_on_hardware;
    struct list_head post_hardware;

    // covers transfers_done, do-not attempt to take the hardware_lock after
    // taking this lock as that can deadlock.
    spinlock_t done_lock;
    struct list_head transfers_done;

    struct dma_pool* descriptor_pool;
    struct workqueue_struct* finishing_queue;
    struct work_struct finishing_work;

    // DMA descriptor terminating the descriptor chain
    minion_dma_extdesc_t* terminal_desc;
    dma_addr_t terminal_desc_phys;

    // source of sequence-numbers, use hardware_lock when modifying
    u16 sequence_no; // for us to track transfers.

    // dma in flight
    atomic_t count;
};
#endif // ONT_ALTERA_DMA_H
