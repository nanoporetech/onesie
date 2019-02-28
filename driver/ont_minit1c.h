/**
 * ont_minit1c.h
 *
 * Copyright (C) 2019 Oxford Nanopore Technologies Ltd.
 *
 * Author: <info@nanoporetech.com>
 *
 */

#ifndef ONT_MINIT1C_H
#define ONT_MINIT1C_H

#define ONT_DEBUG
//#define ONT_VERBOSE_DEBUG

// debug macros
#ifdef DPRINTK
    #error
#endif
#ifdef VPRINTK
    #error
#endif
#ifdef ONT_DEBUG
    #define STRINGIFY_(X) #X
    #define STRINGIFY(X) STRINGIFY_(X)
    #define DPRINTK(ARGS...) do {printk(KERN_ERR __FILE__ ":" STRINGIFY(__LINE__)" :" ARGS);} while(0)

    // optionally dump out the majority of register accesses
    #ifdef ONT_VERBOSE_DEBUG
        #define WRITEL(VAL,ADDR) do{u32 val=(VAL); void* addr=(ADDR); printk(KERN_ERR"minit 0x%08x => %p\n",val,addr);writel(val,addr); } while(0)
        static inline u32 myreadl(void* addr) {u32 r=readl(addr);printk(KERN_ERR"minit 0x%08x <= %p\n",r,addr);return r;}
        #define READL(ADDR) myreadl(ADDR)

        #define VPRINTK(ARGS...) do {printk(KERN_ERR __FILE__ ":" STRINGIFY(__LINE__)" :" ARGS);} while(0)
    #endif
#endif
#ifndef DPRINTK
    #define DPRINTK(ARGS...) do {} while(0)
#endif
#ifndef VPRINTK
    #define VPRINTK(ARGS...) do {} while(0)
    #define READL(ADDR) readl(ADDR)
    #define WRITEL(VAL,ADDR) writel(VAL,ADDR)
#endif

#define ONT_DRIVER_NAME     "ont-minit1c"

/* version is major.minor.patch */
#define ONT_DRIVER_VERSION  "0.0.1"

#define ONT_FIRST_MINOR 0

/* set to 5 for potential use with gridion */
#define MINIT_MAX_DEVICES 5

/* BAR numbers */
#define CTRL_BAR 0
#define SPI_BAR 2
#define PCI_BAR 4

#define CTRL_BAR_EXPECTED_SIZE  0x08000000

#define SPI_BAR_EXPECTED_SIZE   64

#define PCI_BAR_EXPECTED_SIZE   0x3b20


struct altr_i2c_dev;
struct altr_dma_dev;
struct i2c_client;
struct minit_data_transfer_s;
struct minit_transfer_status_s;

struct minit_device_s {
    struct pci_dev* pci_device;

    struct altr_i2c_dev* i2c_dev;
    struct i2c_adapter* i2c_adapter;
    struct altr_dma_dev* dma_dev;

    irqreturn_t (*i2c_isr_quick)(int,void*);
    irqreturn_t (*i2c_isr)(int,void*);
    irqreturn_t (*dma_isr_quick)(int,void*);
    irqreturn_t (*dma_isr)(int,void*);

    // used in bh thread to determine which bh to run
    unsigned long had_i2c_irq;
    unsigned long had_dma_irq;

    // protects against simultanous use of the link to the ASIC shared by the
    // acquisition hardware and I2C/EEPROM
    struct mutex link_mtx;

    // bar virtual addresses
    void __iomem* ctrl_bar;
    void __iomem* spi_bar;
    void __iomem* pci_bar;

    int minor_dev_no;
};


extern int borrowed_altr_i2c_probe(struct minit_device_s* base);
extern void borrowed_altr_i2c_remove(struct minit_device_s* m_dev);
extern int altera_sgdma_probe(struct minit_device_s* mdev);
extern void altera_sgdma_remove(struct minit_device_s* mdev);
extern long queue_data_transfer(struct altr_dma_dev*, struct minit_data_transfer_s*, struct file* file);
extern u32 get_completed_data_transfers(struct altr_dma_dev*, u32, struct minit_transfer_status_s*);
extern long cancel_data_transfer(struct altr_dma_dev*, u32);
extern void cancel_data_transfer_for_file(struct altr_dma_dev* , struct file*);
#endif        //  #ifndef ONT_MINIT1C_H
