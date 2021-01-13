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

#include <linux/kobject.h>
#include <linux/sysfs.h>

//#define ONT_DEBUG
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
#define ONT_DRIVER_VERSION  "0.5.3"

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

#define MAX_SET_POINT   ((u16)0x2800) // 40C in 8.8 fixed-point
#define MIN_SET_POINT   ((u16)0x1e00) // 30C in 8.8 fixed-point

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

// group an attribute with a pointer to the value it should communicate
struct attribute_wrapper {
    void* p_value;
    struct kobj_attribute attribute;
};

struct thermal_control_sysfs {
    struct attribute_group thermal_group;
    struct attribute* attributes[13];
    struct attribute_wrapper control;
    struct attribute_wrapper error;
    struct attribute_wrapper tec_override;
    struct attribute_wrapper tec_dead_zone;
    struct attribute_wrapper tec_voltage;
    struct attribute_wrapper tec_current;
    struct attribute_wrapper data_log;
    struct attribute_wrapper latest_data_log;
    struct attribute_wrapper threshold_1;
    struct attribute_wrapper threshold_2;
    struct attribute_wrapper threshold_3;
    struct attribute_wrapper pid_settings;
};

struct shift_reg_access_parameters_s {
    char* to_dev;
    char* from_dev;
    u16* wavetable;
    u16 waveform_length;
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

    struct thermal_control_sysfs tc_attr ;
    u8 last_hs_clk_speed;
    int minor_dev_no;

    // functions for converting Temperature to/from hardware format
    u16 (*temp_to_fixedpoint)(u16 temp);
    u16 (*fixedpoint_to_temp)(u16 temp);
};

/// @todo this is a temporary name for an object who's lifetime matches that
/// of the user keeping the file open
struct minion_user_s {
    struct minion_device_s* mdev;
    /// true if the user did DMA and will trigger a clean-up when closing the file
    bool dma_used;
};


extern int borrowed_altr_i2c_probe(struct minion_device_s* base);
extern void borrowed_altr_i2c_remove(void* ptr);
extern int altera_sgdma_probe(struct minion_device_s* mdev);
extern void altera_sgdma_remove(void* ptr);
extern long queue_data_transfer(struct altr_dma_dev*, struct minion_data_transfer_s*, struct file* file);
extern u32 get_completed_data_transfers(struct altr_dma_dev*, u32, struct minion_transfer_status_s*);
extern long cancel_data_transfers(struct altr_dma_dev*);
extern void dma_dump_debug(struct altr_dma_dev* adma);
#endif        //  #ifndef MINION_TOP_H
