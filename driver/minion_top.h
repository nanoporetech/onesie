/**
 * minion_top.h
 *
 * Copyright (C) 2019 Oxford Nanopore Technologies Ltd.
 *
 * Author: <info@nanoporetech.com>
 *
 */
#ifndef MINION_TOP_H
#define MINION_TOP_H

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
        #define WRITEL(VAL,ADDR) do{u32 val=(VAL); void* addr=(ADDR); printk(KERN_ERR"minion 0x%08x => [%p]\n",val,addr);writel(val,addr); } while(0)
        static inline u32 myreadl(void* addr) {u32 r=readl(addr);printk(KERN_ERR"minion 0x%08x <= [%p]\n",r,addr);return r;}
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

#define ONT_DRIVER_NAME     "ont-minion1c"

/* version is major.minor.patch */
#define ONT_DRIVER_VERSION  "0.0.1"

#define ONT_FIRST_MINOR 0

/* set to 5 for potential use with gridion */
#define MINION_MAX_DEVICES 5

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
struct minion_data_transfer_s;
struct minion_transfer_status_s;

enum link_mode_e {
    link_idle,
    link_mode_data,
    link_mode_i2c
};

struct historical_link_mode {
    enum link_mode_e mode;
    u8 reg;
};

struct shift_reg_access_parameters_s {
    char* to_dev;
    char* from_dev;
    u16* wavetable;
    u16 waveform_length;
    u8 waveform_enable;
    u8 waveform_frames;
    const u8 start;
    const u8 enable;
    const u32 clk;
    u8 cmd_id;
};

struct minion_device_s {
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
    enum link_mode_e link_mode;

    // bar virtual addresses
    void __iomem* ctrl_bar;
    void __iomem* spi_bar;
    void __iomem* pci_bar;

    int minor_dev_no;
};


extern int borrowed_altr_i2c_probe(struct minion_device_s* base);
extern void borrowed_altr_i2c_remove(void* ptr);
extern int altera_sgdma_probe(struct minion_device_s* mdev);
extern void altera_sgdma_remove(void* ptr);
extern long queue_data_transfer(struct altr_dma_dev*, struct minion_data_transfer_s*, struct file* file);
extern u32 get_completed_data_transfers(struct altr_dma_dev*, u32, struct minion_transfer_status_s*);
extern long cancel_data_transfers(struct altr_dma_dev*);
#endif        //  #ifndef MINION_TOP_H
