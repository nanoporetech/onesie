/**
 * ont_minit1c_reg.h
 *
 * Copyright (C) 2019 Oxford Nanopore Technologies Ltd.
 *
 * Author: Richard Crewe <richard.crewe@nanoporetech.com>
 *
 * This file contains register definitions for the MinIT-1C firmware, the sort
 * of stuff that could be generated automatically by tools from the firmware
 * code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef ONT_MINIT1C_REG_H
#define ONT_MINIT1C_REG_H

#define PCIe_LANE_CLOCK 62500000 //62.5 MHz

#define I2C_PARENT_CLOCK_RATE PCIe_LANE_CLOCK
#define EEPROM_CLOCK_RATE 390625

/* BASE ADDRESSES BAR-0*/
#define ASIC_CTRL_BASE          0x02001000
#define ASIC_HS_DMA_BASE        0x00000060
#define ASIC_HS_DMA_PREF_BASE   0x00000040
#define ASIC_HS_RECEIVER_BASE   0x02000000
#define I2C_BASE                0x00000000
#define MESSAGE_RAM_BASE        0x01009000
#define ASIC_SHIFT_BASE         0x03000000

/* BASE ADDRESSES BAR-2*/
#define ADC_SPI_BASE            0x00000000
#define DAC_SPI_BASE            0x00000020


/* BASE ADDRESSES BAR-3 */
#define PCI_ISR                 0x0040
#define PCI_ENB                 0x0050

#define PCI_ISR_I2C             (1 << 0)
#define PCI_ISR_DMA             (1 << 1)

#endif        //  #ifndef ONT_MINIT1C_REG_H
