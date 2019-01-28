/**
 * ont_minit1c.h
 *
 * Copyright (C) 2019 Oxford Nanopore Technologies Ltd.
 *
 * Author: Richard Crewe <richard.crewe@nanoporetech.com>
 *
 */

#ifndef ONT_MINIT1C_H
#define ONT_MINIT1C_H

#define ONT_DRIVER_NAME     "ont-minit1c"

/* version is major.minor.patch */
#define ONT_DRIVER_VERSION  "0.0.1"

#define ONT_FIRST_MINOR 0

/* set to 5 for potential use with gridion */
#define MINIT_MAX_DEVICES 5

/* BAR numbers */
#define CTRL_BAR 0
#define SPI_BAR 2
#define PCI_BAR 3

#define CTRL_BAR_EXPECTED_SIZE  0x01009400

#define SPI_BAR_EXPECTED_SIZE   64

#define PCI_BAR_EXPECTED_SIZE   0x3b20


struct altr_i2c_dev;
struct altr_dma_dev;

struct minit_device_s {
    struct pci_dev* pci_device;

    struct altr_i2c_dev* i2c_dev;
    struct altr_dma_dev* dma_dev;

    irqreturn_t (*i2c_isr_quick)(int,void*);
    irqreturn_t (*i2c_isr)(int,void*);
    irqreturn_t (*dma_isr_quick)(int,void*);
    irqreturn_t (*dma_isr)(int,void*);

    // used in bh thread to determine which bh to run
    unsigned long had_i2c_irq;
    unsigned long had_dma_irq;

    // bar virtual addresses
    void __iomem* ctrl_bar;
    void __iomem* spi_bar;
    void __iomem* pci_bar;

    int minor_dev_no;
};


extern int borrowed_altr_i2c_probe(struct minit_device_s* base);


#endif        //  #ifndef ONT_MINIT1C_H
