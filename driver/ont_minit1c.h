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

#define CTRL_BAR_EXPECTED_SIZE  0x01009400


#define SPI_BAR_EXPECTED_SIZE   64


struct minit_device_s {
    struct pci_dev* pci_device;

    // bar virtual addresses
    void __iomem* ctrl;
    void __iomem* spi;

    int minor_dev_no;
};

#endif        //  #ifndef ONT_MINIT1C_H
