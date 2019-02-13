/**
 * ont_pci_minit_ioctl.h
 *
 * Copyright (C) 2016 Oxford Nanopore Technologies Ltd.
 *
 * Author: Richard Crewe <richard.crewe@nanoporetech.com>
 *
 * These are the IOCTL definition for interacting with the Oxford Nanopore 
 * Technologies Base-caller On A Stick.
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
#ifndef ONT_MINIT_IOCTL_H
#define ONT_MINIT_IOCTL_H

// Hide the __user pointer modifier when not compiled in the kernel
#ifndef __KERNEL__
#define __user
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/ioctl.h>
#include <linux/types.h>

/**
 * @brief register read/write ioctl
 * offset   to-driver   offset of the register from the start of the bar
 * size     to-driver   size of transfer in bytes (1,2,4 or 8)
 * write    to-driver   will write to the register if non-zero
 * bar      to-driver   selects the BAR containing the register
 * value    to/from-driver data to write to the register or the data read from
 *                      the register
 */
struct minit_register_s {
    __u32 offset;
    __u16 size;
    __u8 write;
    __u8 bar;
    __u64 value;
} __attribute__(( packed ));

#define MINIT_IOCTL_REG_ACCESS _IOWR('b', 64, struct minit_register_s)

/**
 * @brief read/write data to the ASICs shift-register for sending commands and
 * reading the current ASIC configuration and OTP bits, etc.
 *
 * to_device to-driver  Pointer to a 282-byte buffer containing data to write
 *                      to the ASIC shift-register. If null existing hardware
 *                      buffer contents will be re-sent.
 * from_device to-driver Pointer to a 282-byte buffer for data read from the
 *                      ASIC shift-register. This can be the same buffer as the
 *                      one pointed to by to_device. If null any data read from
 *                      the hardware will be discarded.
 * clock_hz to-driver   Clock speed in Hz for the transfer. Due to hardware
 *                      limitations the actual clock speed may differ from that
 *                      requested
 * start    to-driver   Sets start-bit if non-zero
 * enable   to-driver   Sets enable-bin it non-zero
 */
struct minit_shift_reg_s {
    void __user *to_device;
    void __user *from_device;
    __u32   clock_hz;
    __u8    start;
    __u8    enable;
};

#define MINIT_IOCTL_SHIFT_REG _IOWR('b', 65, struct minit_shift_reg_s)


#define MINIT_IOCTL_HS_RECEIVER_REG_SIZE 10
/**
 * @brief Optionally write data, then read the contents of the HS Receiver core
 * registers to/from-driver If write is set then the writable registers will be
 *                      written, then all registers will be read and their
 *                      contents copied into these registers.
 * write    to-driver   If non-zero then contents of the registers array will
 *                      be written to writable registers. Which registers are
 *                      writable is programmed into the driver.
 */
struct  minit_hs_receiver_s {
   __u16    registers[MINIT_IOCTL_HS_RECEIVER_REG_SIZE];
   __u8     write;
};

#define MINIT_IOCTL_HS_RECIEVER _IOWR('b', 66, struct minit_hs_receiver_s)

/**
 * @brief Transfer data to/from the EEPROM
 */
struct minit_eeprom_transfer_s {
    char __user*    data;
    __u32           start;
    __u32           length;
};

/**
 * For these defines the return values may indicate
 *  EBUSY   The EEPROM shares electrical connections with the ASIC acquisition
 *          data. If the flow-cell is acquiring data these will return EBUSY
 *  ENODEV  No flow-cell present (the EEPROM is on the flow-cell)
 *  EIO     Probably timed out
 *  ERANGE  Start and/or length are too big for the EEPROM
 */
#define MINIT_IOCTL_EEPROM_READ  _IOWR('b', 67, struct minit_eeprom_transfer_s)
#define MINIT_IOCTL_EEPROM_WRITE _IOWR('b', 68, struct minit_eeprom_transfer_s)

/**
 * @brief The minit_data_transfer_s struct
 * Use to submit transfers to the driver. When these are done, the driver will
 * send the process that submitted the transfer the signal [signal_number]. The
 * user-space process should then send a [MINIT_IOCTL_WHATS_COMPLETED] ioctl
 * to get a list of transfers that are done.
 */
struct minit_data_transfer_s {
    char __user*    buffer;
    __u32           buffer_size;
    __u32           transfer_id;
    int             signal_number;
    int             pid;
};
#define MINIT_IOCTL_SUBMIT_TRANSFER  _IOWR('b', 69, struct minit_data_transfer_s)

/**
 * @brief The minit_transfer_status struct
 */
struct minit_transfer_status_s {
    __u32           transfer_id;
    __u32           status;// 0 = OK
};

/**
 * @brief For fetching information about completed transfers.
 *
 * [completed_transfers] should point to a buffer with space for
 * [completed_transfers_size] elements. This will be filled in by the driver
 * with the status of the transfers.
 */
struct minit_completed_transfers_s {
    struct minit_transfer_status_s*
                    completed_transfers;
    __u32           completed_transfers_size;

    /**
     * if this is returned with a number matching [completed_transfers_size]
     * then there may be more transfers waiting, check again. */
    __u32           no_completed_transfers;
};

/**
 * After receiving a signal from the driver, the user-space code uses this
 * ioctl to enquire which transfers are done. When this IOCTL returns, the
 * transfers for which status is provided are completely disowned by the driver.
 */
#define MINIT_IOCTL_WHATS_COMPLETED  _IOWR('b', 70, struct minit_completed_transfers_s)

/** Cancel a transfer, the argument is the transfer_id */
#define MINIT_IOCTL_CANCEL_TRANSFER  _IOWR('b', 71, __u32)

#ifdef __cplusplus
}
#endif

#endif        //  #ifndef ONT_MINIT_IOCTL_H
