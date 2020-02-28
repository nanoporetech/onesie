/**
 * minion_reg.h
 *
 * Copyright (C) 2019 Oxford Nanopore Technologies Ltd.
 *
 * Author: <info@nanoporetech.com>
 *
 * This file contains register definitions for the MinION-1C firmware, the sort
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

#ifndef MINION_REG_H
#define MINION_REG_H

#define PCIe_LANE_CLOCK 62500000 //62.5 MHz

#define I2C_PARENT_CLOCK_RATE PCIe_LANE_CLOCK
#define EEPROM_CLOCK_RATE 390625
#define EEPROM_SIZE 256 // its 256 bytes, even though half is mostly zeros
#define EEPROM_WRITABLE_SIZE 128 // can only write to the bottom half of the EEPROM
#define EEPROM_ADDRESS 0x50

/* BASE ADDRESSES BAR-0*/
#define I2C_BASE                0x00000000
#define ASIC_HS_DMA_PREF_BASE   0x00000040
#define ASIC_HS_DMA_BASE        0x00000060
#define SYSTEM_ID_CORE          0x00000080
#define NIOS_MESSAGE_RAM_BASE   0x00010000
#define NIOS_MUTEX_BASE         0x00012000
#define ASIC_CTRL_BASE          0x00020000
#define ASIC_SHIFT_BASE         0x00020800
#define ASIC_HS_RECEIVER_BASE   0x00021000

/* system-id core */
#define SYS_ID_VERSION          0x0
#define SYS_ID_VERSION_MAJOR_S  16
#define SYS_ID_VERSION_MINOR_S  8
#define SYS_ID_TIMESTAMP        0x4

/* ASIC Control Bits and masks */
#define ASIC_CTRL 0
#define ASIC_CTRL_RESET         (1 << 0)
#define ASIC_CTRL_ALG_POWER     (1 << 1)
#define ASIC_CTRL_CLK_MASK      (3 << 2)
#define ASIC_CTRL_CLK_128       (1 << 2)
#define ASIC_CTRL_CLK_64        (2 << 2)
#define ASIC_CTRL_CLK_32        (3 << 2)
#define ASIC_CTRL_CLK_SHIFT     2
#define ASIC_CTRL_BUS_MODE      (1 << 4)
#define ASIC_CTRL_DETECT        (1 << 5)
#define ASIC_CTRL_GOOD_POWER    (1 << 7)
#define ASIC_CTRL_CLK_FBK_SHIFT 8
#define ASIC_CTRL_CLK_FBK_MASK  0x3f00
#define ASIC_CTRL_HWID_MASK     (3 << 14)
#define ASIC_CTRL_HWID_SHIFT    14

#define ASIC_CTRL2 2
#define ASIC_CTRL2_CLK_MASK     (1 << 0)
#define ASIC_CTRL2_CLK_SHIFT    0

/* ASIC shift register; buffers and control register */
#define ASIC_SHIFT_REG_SIZE     0x11b
#define ASIC_SHIFT_OUTPUT_BUF   0
#define ASIC_SHIFT_INPUT_BUF    0x180
#define ASIC_SHIFT_CTRL         0x300
#define ASIC_SHIFT_WAVE_CTRL1   0x302
#define ASIC_SHIFT_LUT_LEN_MASK 0x01FF
#define ASIC_SHIFT_LUT_ENABLE   (1 << 15)

#define ASIC_SHIFT_WAVE_CTRL2   0x304
#define ASIC_SHIFT_WAVE_FRAMES_MASK 0x007F

#define ASIC_SHIFT_WAVE_TABLE   0x400

#define ASIC_SHIFT_CTRL_EN      (1 << 0)
#define ASIC_SHIFT_CTRL_ST      (1 << 1)
#define ASIC_SHIFT_CTRL_DIV_MASK  0x3f
#define ASIC_SHIFT_CTRL_DIV_MAX   0x3e
#define ASIC_SHIFT_CTRL_DIV_SHIFT 2
#define ASIC_SHIFT_CTRL_CMDID_SHIFT 8

#define ASIC_SHIFT_DIV_TO_CLOCK(DIV) (PCIe_LANE_CLOCK / ((4*(DIV)) + 2))
#define ASIC_SHIFT_MAX_CLOCK ASIC_SHIFT_DIV_TO_CLOCK(0)
#define ASIC_SHIFT_MIN_CLOCK ASIC_SHIFT_DIV_TO_CLOCK(ASIC_SHIFT_CTRL_DIV_MAX)


/* ASIC HS-RX REGISTERS */
#define NUM_HS_REGISTERS 13

#define ASIC_HS_RECEIVER_REMAP  0x400

/* bitmask of HS Reveiver registers that can be written */
#define ASIC_HS_REG_WRITE_MASK  ((1 << 0) | (1 << 9) | (1 << 0xb) | (1 << 0xc))
/* BASE ADDRESSES BAR-2*/
#define ADC_SPI_BASE            0x00012000
#define DAC_SPI_BASE            0x00012020


/* BASE ADDRESSES BAR-4 */
/* These are in a PCIe CRA core at offset-0 in the bar */
#define PCI_ISR                 0x0040
#define PCI_ENB                 0x0050

/* BAR-3 interrup status and enableb register bits */
#define PCI_ISR_I2C             (1 << 1)
#define PCI_ISR_DMA             (1 << 0)





#endif        //  #ifndef MINION_REG_H
