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

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/ioctl.h>
#include <linux/types.h>

struct minit_register_s {
    __u32 offset;
    __u16 size;
    __u8 write;
    __u8 bar;
    __u64 value;
} __attribute__(( packed ));


#define MINIT_IOCTL_REG_ACCESS _IOWR('b', 64, struct minit_register_s)

#ifdef __cplusplus
}
#endif

#endif        //  #ifndef ONT_MINIT_IOCTL_H
