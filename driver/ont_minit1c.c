/**
 * ont_minit1c.c
 *
 * Copyright (C) 2019 Oxford Nanopore Technologies Ltd.
 *
 * Author: Richard Crewe <richard.crewe@nanoporetech.com>
 *
 * This file contains register definitions for the MinIT-1C firmware
 *
 */

//#define ONT_DEBUG

#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>

#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include "ont_minit1c.h"
#include "ont_minit1c_reg.h"
#include "ont_minit_ioctl.h"

/*
 * PROTOTYPES
 */

static int __init pci_probe(struct pci_dev *dev, const struct pci_device_id *id);
static void __exit pci_remove(struct pci_dev *dev);
static int minit_file_open(struct inode* inode, struct file *file);
static int minit_file_close(struct inode *inode, struct file *file);
static long minit_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int switch_link_mode(struct minit_device_s* , const enum link_mode_e , struct historical_link_mode* );
static void free_link(struct minit_device_s* , const struct historical_link_mode* );
/*
 * STRUCTURES
 */
static struct pci_device_id pci_ids[] = {
    { PCI_DEVICE(0x1ab0, 0x0010) }, // Altera development card
    { 0 }
};

static struct pci_driver minit_driver_ops = {
    .name = ONT_DRIVER_NAME,
    .id_table = pci_ids,
    .probe = pci_probe,
    .remove = pci_remove,
};

static struct file_operations minit_fops =
{
    .owner          = THIS_MODULE,
    .open           = minit_file_open,
    .unlocked_ioctl = minit_unlocked_ioctl,
    .release        = minit_file_close,
};

/*
 * MODULE GLOBALS
 */
static struct class* minit_class = NULL;
static int minit_major = 0;
static int minit_minor = 0;
static struct cdev minit_cdev = {};

/* Array of minit_device pointers indexed by device-node minor number */
static struct minit_device_s* minit_device_table[MINIT_MAX_DEVICES] = {};


/*
 * FUNCTIONS
 */

/**
 * @return <0 on error, 0 on OK
 */
static int device_table_add(unsigned int minor, struct minit_device_s* minit_dev)
{
    VPRINTK("device_table_add\n");
    if (minor >= MINIT_MAX_DEVICES) {
        printk(KERN_ERR ONT_DRIVER_NAME":Too many devices\n");
        return -EINVAL;
    }

    if (minit_device_table[minor]) {
        printk(KERN_ERR ONT_DRIVER_NAME":That device node is already in use\n");
        return -EEXIST;
    }

    DPRINTK("Adding %p at %d\n", minit_dev, minor);
    minit_device_table[minor] = minit_dev;
    return 0;
}

/**
 * @return <0 on error, minor number of removed device will be returned if OK
 */
static int device_table_remove(struct minit_device_s* minit_dev)
{
    int index;
    VPRINTK("device_table_remove\n");
    for (index = 0; index < MINIT_MAX_DEVICES; ++index) {
        if (minit_device_table[index] == minit_dev) {
            DPRINTK("Removing entry %p at %d\n",minit_dev, index);
            minit_device_table[index] = 0;
            return index;
        }
    }
    DPRINTK("%p not found in device table\n",minit_dev);

    return -ENOENT;
}

static struct minit_device_s* device_table_lookup(unsigned int minor)
{
    VPRINTK("device_table_lookup\n");
    if (minor >= MINIT_MAX_DEVICES) {
        printk(KERN_ERR ONT_DRIVER_NAME":invalid device number %d\n",minor);
        return NULL;
    }

    return minit_device_table[minor];
}

/**
 * This performs the register reading and writing for IOCTLs. It bound checks
 * register offsets, but does not enforse alignment
 * @param minit_dev device in which to access the register
 * @param reg_access contains details of the register access and will be
 *     modified with read values
 * @return 0 on success
 */
static long minit_reg_access(struct minit_device_s* minit_dev, struct minit_register_s* reg_access)
{
    long rc = 0;
    void* address;

    BUILD_BUG_ON(sizeof(struct minit_register_s) != 16);
    if (unlikely(!minit_dev)) {
        printk(KERN_ERR ONT_DRIVER_NAME": ioctl called when there is no hardware.\n");
        return -ENODEV;
    }

    /* select bar and check for out of bounds accesses */
    switch (reg_access->bar) {
    case CTRL_BAR:
        address = minit_dev->ctrl_bar;
        if ((reg_access->offset + reg_access->size) > CTRL_BAR_EXPECTED_SIZE) {
            return -EFAULT;
        }
        break;
    case SPI_BAR:
        address = minit_dev->spi_bar;
        if ((reg_access->offset + reg_access->size) > SPI_BAR_EXPECTED_SIZE) {
            return -EFAULT;
        }
        break;
    case PCI_BAR:
        address = minit_dev->pci_bar;
        if ((reg_access->offset + reg_access->size) > PCI_BAR_EXPECTED_SIZE) {
            return -EFAULT;
        }
        break;
    default:
        printk(KERN_ERR ONT_DRIVER_NAME": Invalid BAR %d\n", reg_access->bar);
        return -EFAULT;
        break;
    }
    address += reg_access->offset;

    if (reg_access->write) {
        // perform writes of various sizes
        switch(reg_access->size) {
        case 1:
            VPRINTK("write-8 0x%02x -> %p\n", (u8)reg_access->value, address);
            writeb((u8)reg_access->value, address);
            break;
        case 2:
            VPRINTK("write-16 0x%04x -> %p\n", (u16)reg_access->value, address);
            writew((u16)reg_access->value, address);
            break;
        case 4:
            VPRINTK("write-32 0x%08x -> %p\n", (u32)reg_access->value, address);
            WRITEL((u32)reg_access->value, address);
            break;
        case 8:
            VPRINTK("write-64 0x%016llx -> %p\n", (u64)reg_access->value, address);
            writeq(reg_access->value, address);
            break;
        default:
            printk(KERN_ERR ONT_DRIVER_NAME": Invalid access size %d\n", reg_access->size);
            return -EINVAL;
            break;
        }
    } else {
        // perform reads of various sizes
        switch(reg_access->size) {
        case 1:
            reg_access->value = (u64)readb(address);
            VPRINTK("read-8 0x%02x <- %p\n", (u8)reg_access->value, address);
            break;
        case 2:
            reg_access->value = (u64)readw(address);
            VPRINTK("read-16 0x%04x <- %p\n", (u16)reg_access->value, address);
            break;
        case 4:
            reg_access->value = (u64)READL(address);
            VPRINTK("read-32 0x%08x <- %p\n", (u32)reg_access->value, address);
            break;
        case 8:
            reg_access->value = readq(address);
            VPRINTK("read-64 0x%016llx <- %p\n", (u64)reg_access->value, address);
            break;
        default:
            printk(KERN_ERR ONT_DRIVER_NAME": Invalid access size %d\n", reg_access->size);
            return -EINVAL;
            break;
        }
    }

    return rc;
}

/**
 * @brief data transfer and control via shift-register
 * @param minit_dev pointer do driver structure
 * @param to_dev 282 byte buffer to send to ASIC
 * @param from_dev 282 byte buffer to receive data from ASIC
 * @param start  start transfer
 * @param enable enable module
 * @param clk clockspeed in Hz, this will be achieved by integer division of
 * the 62.5 MHz PCIe clock
 * @return
 */
static long minit_shift_register_access(
        struct minit_device_s* minit_dev,
        char* const to_dev,
        char* const from_dev,
        const u8 start,
        const u8 enable,
        const u32 clk)
{
    struct historical_link_mode old_link_mode;
    long rc;
    u32 clockdiv;
    u32 actual_clock;
    u32 control;
    unsigned int delay_ms = 1+((1000 * ASIC_SHIFT_REG_SIZE) / clk);
    VPRINTK("minit_shift_register_access to_dev %p, from_dev %p, start %d, enable %d, clk %d\n",
            to_dev, from_dev, start, enable, clk);

    rc = switch_link_mode(minit_dev, link_mode_data, &old_link_mode);
    if (rc < 0) {
        return rc;
    }

    // write to data into shift register
    if (to_dev) {
        int i;
        for (i = 0; i < ASIC_SHIFT_REG_SIZE; ++i) {
            VPRINTK("Shift to dev  : 0x%02x => %p\n",from_dev[i],minit_dev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_OUTPUT_BUF + i);
            writeb(to_dev[i], minit_dev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_OUTPUT_BUF + i);
        }
    }
    wmb();

    if (clk > PCIe_LANE_CLOCK) {
        clockdiv = 0;
    } else {
        clockdiv = ((PCIe_LANE_CLOCK/clk) - 1) / 2;
    }
    if (clockdiv > ASIC_SHIFT_CTRL_DIV_MAX) {
        clockdiv = ASIC_SHIFT_CTRL_DIV_MAX;
    }
    actual_clock = PCIe_LANE_CLOCK / ((2*clockdiv) + 1);
    if (actual_clock != clk) {
        DPRINTK("Requested SPI Clock of %d couldn't be achieved, best effort %d\n",
                clk, actual_clock);
    }

    control = (clockdiv << ASIC_SHIFT_CTRL_DIV_SHIFT) |
              (start ? ASIC_SHIFT_CTRL_ST : 0) |
              (enable ? ASIC_SHIFT_CTRL_EN :0);
    VPRINTK("shift reg control 0x%02x => %p\n", control, minit_dev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_CTRL);
    writeb(control, minit_dev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_CTRL);
    wmb();

    if (from_dev) {
        int i;
        // wait for the data to move
        VPRINTK("sleeping for %d ms\n",delay_ms);
        msleep(delay_ms);

        for (i = 0; i < ASIC_SHIFT_REG_SIZE; ++i) {
            from_dev[i] = readb(minit_dev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_INPUT_BUF + i);
            VPRINTK("Shift from dev: 0x%02x <= %p\n",from_dev[i],minit_dev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_INPUT_BUF + i);
        }
    }

    free_link(minit_dev, &old_link_mode);

    return 0;
}

static void minit_hs_reg_access(struct minit_device_s* minit_dev, struct minit_hs_receiver_s* minit_hs_reg)
{
    unsigned int i;


    DPRINTK("minit_hs_reg_access\n");
    if (minit_hs_reg->write) {
        for (i = 0; i < NUM_HS_REGISTERS;++i) {
            if (((ASIC_HS_REG_WRITE_MASK >> i) & 1) == 1) {
                writew(minit_hs_reg->registers[i], minit_dev->ctrl_bar + ASIC_HS_RECEIVER_BASE + (i * 2));
            }
        }

    }

    // now read registers
    for (i = 0; i < NUM_HS_REGISTERS;++i) {
        minit_hs_reg->registers[i] = readw(minit_dev->ctrl_bar + ASIC_HS_RECEIVER_BASE + (i * 2));
    }
}


/*
 * want to lock-out other modes until done, but allow re-entrance for things running the same mode
 */
static int switch_link_mode(struct minit_device_s* mdev, const enum link_mode_e mode, struct historical_link_mode* old_mode)
{
    int rc = 0;
    u8 asic_ctrl;
    // check the link wires are not being used
    VPRINTK("switch_link_mode %s\n", mode == link_mode_i2c ? "i2c" : "data");

    // can't switch to idle, use free_link()
    if (unlikely(mode == link_idle)) {
        BUG();
        return -EINVAL;
    }

    if (mutex_trylock(&mdev->link_mtx) == 0) {
        return -EBUSY;
    }

    // if not idle then exit, busy
    if (mdev->link_mode != link_idle) {
        rc = -EBUSY;
        goto out;
    }

    asic_ctrl = readb(mdev->ctrl_bar + ASIC_CTRL_BASE) ;
    old_mode->reg = asic_ctrl;
    old_mode->mode = mdev->link_mode;
    mdev->link_mode = mode;

    // read and modify the asic-control register to set either I2C or SPI mode
    switch(mode & ASIC_CTRL_MASK) {
    case link_mode_i2c:
        asic_ctrl |= ASIC_CTRL_BUS_MODE | ASIC_CTRL_RESET ;
        break;
    case link_mode_data:
        asic_ctrl &= ~(ASIC_CTRL_BUS_MODE | ASIC_CTRL_RESET );
        asic_ctrl |= ASIC_CTRL_ALG_POWER;
        break;
    }
    writeb(asic_ctrl, mdev->ctrl_bar + ASIC_CTRL_BASE);

out:
    mutex_unlock(&mdev->link_mtx);
    return 0;
}

static void free_link(struct minit_device_s* mdev, const struct historical_link_mode* old_mode)
{
    VPRINTK("free_link\n");
    if (unlikely(!old_mode)) {
        printk(KERN_ERR"call to free_link with null mode");
        return;
    }
    mutex_lock(&mdev->link_mtx);
    mdev->link_mode = old_mode->mode;
    writeb(old_mode->reg, mdev->ctrl_bar + ASIC_CTRL_BASE);
    mutex_unlock(&mdev->link_mtx);
}

/**
 * read bytes from a page in the eeprom, all locking set-up and the read won't
 * cross page boundries
 *
 * @param adapter the i2c adapter that will handle the trasaction
 * @param buffer pointer to the buffer of data to read
 * @param start address in the eeprom of the read
 * @param length number of bytes to write (max 8)
 * @return negative on error
 */
static int read_eeprom_page(struct i2c_adapter* adapter, u8* buffer, u8 start, u16 length)
{
    struct i2c_msg read_msgs[2] = {
        // start, write memory address
        {
            .addr = EEPROM_ADDRESS,
            .flags = 0,
            .len = 1,
            .buf = &start
        },
        // start again, read data and stop
        {
            .addr = EEPROM_ADDRESS,
            .flags = I2C_M_RD,
            .len = length,
            .buf = buffer
        }
    };

    VPRINTK("read_eeprom_page buffer %p, start %d, len %d\n", buffer, start, length);
    return i2c_transfer(adapter, read_msgs, 2);
}

/**
 * write bytes to a page in the eeprom, all locking set-up and the read won't
 * cross page boundries. This may return before the data has been commited to
 * the eeprom, use write_done() to check the write is complete
 *
 * @param adapter the i2c adapter that will handle the trasaction
 * @param buffer pointer to the buffer of data to write
 * @param start address in the eeprom of the write
 * @param length number of bytes to write (max 8)
 * @return negative on error
 */
static int write_eeprom_page(struct i2c_adapter* adapter, u8* buffer, u8 start, u16 length)
{
    int i;
    u8 lendata[9] = {0};

    struct i2c_msg write_msg = {
        // start, write memory address and data, then stop
        .addr = EEPROM_ADDRESS,
        .flags = 0,
        .len = 1 + length,
        .buf = lendata
    };

    VPRINTK("write_eeprom_page buffer %p, start %d, len %d\n", buffer, start, length);
    // have to copy the data to write to concatenate it with the start address
    lendata[0] = start;
    for (i = 0; i < length;++i) {
        lendata[1+i] = buffer[i];
    }

    return i2c_transfer(adapter, &write_msg, 1);
}

/**
 * @brief wait until the last write has been committed to the eeprom
 * @param adapter i2c bus handling the transfer
 * @return negative on error
 */
static int write_done(struct i2c_adapter* adapter)
{
    u8 buf = 0xff;
    struct i2c_msg want_ack_msg = {
        // start, write memory address and data, then stop
        .addr = EEPROM_ADDRESS,
        .flags = 0,
        .len = 1,
        .buf = &buf
    };
    VPRINTK("write_done fn call\n");
    return i2c_transfer(adapter, &want_ack_msg, 1);
}


/**
 * @brief read data from the eeprom, this should check for valid start, length
 * and do the appropriate locking and bus mode changes
 *
 * @param mdev minit device
 * @param buffer pointer to the buffer for the data
 * @param start  start index in eeprom
 * @param length number of bytes to read
 * @return error if negative, 0 if OK
 */
static long read_eeprom(struct minit_device_s* mdev, u8* buffer, u32 start, u32 length)
{
    struct historical_link_mode old_mode;
    int rc=0;
    const u8 eeprom_page_size = 8;

    struct i2c_adapter* adapter = mdev->i2c_adapter;
    VPRINTK("read_eeprom buffer %p, start %d, len %d\n",buffer,start,length);
    if (!adapter) {
        return -ENODEV;
    }

    // check the read fits inside the EEPROM
    if (start >= EEPROM_SIZE || (length + start) > EEPROM_SIZE) {
        return -ERANGE;
    }

    // switch link to i2c mode
    rc = switch_link_mode(mdev, link_mode_i2c, &old_mode);
    if (rc < 0) {
        return rc;
    }

    // work through the data breaking on page-boundaries
    while (length > 0 && rc >= 0) {
        u32 bytes_left_on_page = eeprom_page_size - (start % eeprom_page_size);
        u8 this_read_length = (u8)min(bytes_left_on_page, length);
        rc = read_eeprom_page(adapter, buffer, (u8)start, (u8)this_read_length);
        length -= this_read_length;
        start += this_read_length;
        buffer += this_read_length;
    }

    // switch link out of i2c mode
    free_link(mdev, &old_mode);

    return (rc < 0) ? rc : 0;
}

/**
 * @brief write data to the eeprom, this should check for valid start, length
 * and do the appropriate locking and bus mode changes. When this returns, the
 * EEPROM should be programmed.
 *
 * @param mdev minit device
 * @param buffer pointer to the buffer containing the data
 * @param start  start index in eeprom
 * @param length number of bytes to read
 * @return error if negative, 0 if OK
 */
static long write_eeprom(struct minit_device_s* mdev, u8* buffer, u32 start, u32 length)
{
    int retry_count;
    struct historical_link_mode old_mode;
    int rc=0;
    const u8 eeprom_page_size = 8;
    struct i2c_adapter* adapter = mdev->i2c_adapter;
    VPRINTK("write_eeprom buffer %p, start %d, len %d\n",buffer,start,length);
    if (!adapter) {
        return -ENODEV;
    }

    // check the write fits inside the writable bit of the EEPROM
    if (start >= EEPROM_WRITABLE_SIZE || (length + start) > EEPROM_WRITABLE_SIZE) {
        return -ERANGE;
    }

    // switch link to i2c mode
    rc = switch_link_mode(mdev, link_mode_i2c, &old_mode);
    if (rc < 0) {
        return rc;
    }

    // work through the data breaking on page-boundaries
    while (length > 0 && rc >= 0) {
        u32 bytes_left_on_page = eeprom_page_size - (start % eeprom_page_size);
        u8 this_write_length = (u8)min(bytes_left_on_page, length);
        retry_count = 10;
        do {
            rc = write_eeprom_page(adapter, buffer, (u8)start, (u8)this_write_length);
            // the eeprom will not acknowledge further comms until it's finished
            // it's last write, so errors are likely, retry in a few ms
            if (rc) {
                msleep(5);
            }
        } while (--retry_count && rc);
        length -= this_write_length;
        start += this_write_length;
        buffer += this_write_length;
    }
    // after the last write, wait for the write to complete
    retry_count = 10;
    rc = write_done(adapter);
    while (--retry_count && rc) {
        msleep(5);
        rc = write_done(adapter);
    }

    // switch link out of i2c mode
    free_link(mdev, &old_mode);

    return (rc < 0) ? rc : 0;
}


/*
 * File OPs
 */

static int minit_file_open(struct inode* inode, struct file *file)
{
    struct minit_device_s* minit_dev;

    VPRINTK("minit_file_open\n");
    /* work out which hardware the user is expecting to talk to from the device no */
    minit_dev = device_table_lookup(iminor(inode));
    if (!minit_dev) {
        printk(KERN_ERR ONT_DRIVER_NAME":No MINIT device with minor device number %d\n",iminor(inode));
        return -ENODEV;
    }

    file->private_data = minit_dev;
    return 0;
}

/**
 * @TODO: fill this in
 */
static int minit_file_close(struct inode *inode, struct file *file)
{
    VPRINTK("minit_file_close\n");
    return 0;
}

static long minit_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long rc = 0;
    struct minit_device_s* minit_dev = file->private_data;

    VPRINTK("IOCTL file %p cmd %u, arg %lu\n",file, cmd, arg);
    VPRINTK("minit_dev = %p\n",minit_dev);
    if (_IOC_TYPE(cmd) != 'b') {
        DPRINTK("bad magic number\n");
        return -ENOTTY;
    }

    switch(cmd) {
    case MINIT_IOCTL_REG_ACCESS: {
            struct minit_register_s reg_access = {};
            VPRINTK("MINIT_IOCTL_REG_ACCESS\n");
            rc = copy_from_user(&reg_access, (void __user*)arg, sizeof(struct minit_register_s) );
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
            }
            rc = minit_reg_access(minit_dev, &reg_access);
            if (rc) {
                DPRINTK("minit_reg_access failed\n");
                return rc;
            }
            return copy_to_user((void __user*)arg, &reg_access, sizeof(struct minit_register_s) );
        }
        break;
    case MINIT_IOCTL_SHIFT_REG: {
            struct minit_shift_reg_s shift_reg_access = {};
            char shift_reg[ASIC_SHIFT_REG_SIZE];
            VPRINTK("MINIT_IOCTL_SHIFT_REG\n");
            rc = copy_from_user(&shift_reg_access, (void __user*)arg, sizeof(struct minit_shift_reg_s));
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
            }
            if (shift_reg_access.to_device) {
                rc = copy_from_user(shift_reg, shift_reg_access.to_device, ASIC_SHIFT_REG_SIZE);
                if (rc) {
                    DPRINTK("copy_from_user failed\n");
                    return rc;
                }
            }

            // do access
            rc = minit_shift_register_access(
                        minit_dev,
                        shift_reg_access.to_device ? shift_reg : NULL,
                        shift_reg_access.from_device ? shift_reg : NULL,
                        shift_reg_access.start,
                        shift_reg_access.enable,
                        shift_reg_access.clock_hz);
            if (rc) {
                DPRINTK("shift register operation failed\n");
                return rc;
            }
            if (shift_reg_access.from_device) {
                rc = copy_to_user(shift_reg_access.from_device, shift_reg, ASIC_SHIFT_REG_SIZE);
                if (rc) {
                    DPRINTK("copy_to_user failed\n");
                    return rc;
                }
            }
            return copy_to_user((void __user*)arg, &shift_reg_access, sizeof(struct minit_shift_reg_s) );
        }
        break;
    case MINIT_IOCTL_HS_RECIEVER: {
            struct minit_hs_receiver_s minit_hs_reg = {};
            VPRINTK("MINIT_IOCTL_HS_RECIEVER\n");
            rc = copy_from_user(&minit_hs_reg, (void __user*)arg, sizeof(minit_hs_reg));
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
            }
            minit_hs_reg_access(minit_dev, &minit_hs_reg);

            return copy_to_user((void __user*) arg, &minit_hs_reg, sizeof(minit_hs_reg));
        }
        break;
    case MINIT_IOCTL_EEPROM_READ: {
            struct minit_eeprom_transfer_s transfer;
            u8 k_buffer[EEPROM_SIZE] = {0};
            VPRINTK("MINIT_IOCTL_EEPROM_READ\n");
            rc = copy_from_user(&transfer, (void __user*)arg, sizeof(transfer));
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
            }
            rc = read_eeprom(minit_dev, k_buffer, transfer.start, transfer.length );
            if (rc) {
                DPRINTK("eeprom access failed\n");
                return rc;
            }
            // transfer.length is assumed to have been checked for a safe size
            // in read_eeprom
            rc = copy_to_user(transfer.data, k_buffer, transfer.length);
            if (rc) {
                DPRINTK("failed to copy eeprom back into userspace\n");
                return rc;
            }
            return 0;
        }
    case MINIT_IOCTL_EEPROM_WRITE: {
        struct minit_eeprom_transfer_s transfer;
        u8 k_buffer[EEPROM_SIZE] = {0};
        VPRINTK("MINIT_IOCTL_EEPROM_WRITE\n");
        rc = copy_from_user(&transfer, (void __user*)arg, sizeof(transfer));
        if (rc) {
            DPRINTK("copy_from_user failed\n");
            return rc;
        }
        if (transfer.length > EEPROM_SIZE) {
            DPRINTK("EEPROM write size of %d exceeds maximum of %d",
                    transfer.length,
                    EEPROM_SIZE);
        }
        rc = copy_from_user(k_buffer, transfer.data, transfer.length);
        if (rc) {
            DPRINTK("copy_from_user failed to get data from user-space\n");
            return rc;
        }
        rc = write_eeprom(minit_dev, k_buffer, transfer.start, transfer.length );
        if (rc) {
            DPRINTK("eeprom access failed\n");
            return rc;
        }
        return 0;
    }
    default:
        printk(KERN_ERR ONT_DRIVER_NAME": Invalid ioctl for this device (%u)\n", cmd);
    }
    return -EIO;
}


static void cleanup_device(void* data) {
    struct pci_dev* dev = (struct pci_dev*)data;
    struct minit_device_s* minit_dev = pci_get_drvdata(dev);
    VPRINTK("cleanup_device\n");
    if (minit_dev) {
        borrowed_altr_i2c_remove(minit_dev);
        device_table_remove(minit_dev);
        device_destroy(minit_class, MKDEV(minit_major, minit_dev->minor_dev_no));
    }
}

static irqreturn_t minit_isr_quick(int irq, void* _dev)
{
    u32 isr;
    struct minit_device_s* minit_dev = _dev;
    irqreturn_t ret = IRQ_NONE;

    VPRINTK("minit_isr_quick\n");
    isr = READL(minit_dev->pci_bar + PCI_ISR);

    // call the quick ISR for the two cores that can generate interrupts
    if (minit_dev->i2c_isr_quick/* &&
        (isr & PCI_ISR_I2C)*/ )
    {
        ret |= minit_dev->i2c_isr_quick(irq, minit_dev->i2c_dev);
        set_bit(0, &minit_dev->had_i2c_irq);
    }

    if (minit_dev->dma_isr_quick/* &&
        (isr & PCI_ISR_DMA)*/ )
    {
        ret |= minit_dev->dma_isr_quick(irq, minit_dev->dma_dev);
        set_bit(0, &minit_dev->had_dma_irq);
    }

    return (ret & IRQ_WAKE_THREAD ? IRQ_WAKE_THREAD : ret);
}

static irqreturn_t minit_isr(int irq, void* _dev)
{
    struct minit_device_s* minit_dev = _dev;
    irqreturn_t ret = IRQ_NONE;

    VPRINTK("minit_isr (bh)\n");
    // should only get an IRQ from I2C core if it has a message
    if (minit_dev->i2c_isr && test_and_clear_bit(0,&minit_dev->had_i2c_irq)) {
        ret |= minit_dev->i2c_isr(irq, minit_dev->i2c_dev);
    }
    if (minit_dev->dma_isr && test_and_clear_bit(0,&minit_dev->had_dma_irq)) {
        ret |= minit_dev->dma_isr(irq, minit_dev->dma_dev);
    }
    return ret;
}

/**
 * This should:
 *  Verify that we're talking to usable minit hardware.
 *  Map bars and create the device data structure.
 *
 */
static int __init pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    int rc,irq;
    struct minit_device_s* minit_dev = NULL;
    struct device* char_device;
    DPRINTK("PROBE\n");

    // register additional cleanup code for non-managed driver resources
    devm_add_action(&dev->dev, cleanup_device, dev);


    // wake the device up (will be disabled by managed resources on remove/error)
    rc = pcim_enable_device(dev);
    if (rc) {
        dev_err(&dev->dev, "pcim_enable_device() failed\n");
        goto err;
    }

    // allocate and initialise device structure
    minit_dev = devm_kzalloc(&dev->dev, sizeof(struct minit_device_s), GFP_KERNEL);
    if (!minit_dev) {
        dev_err(&dev->dev, "Unable to allocate memory for managing device\n");
        goto err;
    }
    minit_dev->link_mode = link_idle;
    mutex_init(&minit_dev->link_mtx);
    minit_dev->pci_device = dev;
    minit_dev->minor_dev_no = minit_minor++;
    pci_set_drvdata(dev, minit_dev);

    // map all bars
    rc = pcim_iomap_regions(dev, 1 << CTRL_BAR, "MinION control");
    if (rc) {
        dev_err(&dev->dev, "Failed to claim the control BAR\n");
        goto err;
    }
    rc = pcim_iomap_regions(dev, 1<< SPI_BAR, "MinION SPI");
    if (rc < 0) {
        dev_err(&dev->dev, "Failed to claim the SPI BAR\n");
        goto err;
    }
    rc = pcim_iomap_regions(dev, 1<< PCI_BAR, "MinION PCI-Interface");
    if (rc < 0) {
        dev_err(&dev->dev, "Failed to claim the PCI BAR\n");
        goto err;
    }

    minit_dev->ctrl_bar  = pcim_iomap_table(dev)[CTRL_BAR];
    minit_dev->spi_bar = pcim_iomap_table(dev)[SPI_BAR];
    minit_dev->pci_bar = pcim_iomap_table(dev)[PCI_BAR];
    DPRINTK("Control bar mapped to %p\n", minit_dev->ctrl_bar);
    DPRINTK("SPI bar mapped to %p\n", minit_dev->spi_bar);
    DPRINTK("PCI bar mapped to %p\n", minit_dev->pci_bar);

    // Use the existance of a define as indication that we're compiling on a new
    // kernel with the simpler way of setting up interrupts for PCIe
#ifdef PCI_IRQ_ALL_TYPES
    // can use newer simpler way of setting-up irqs for PCI
    rc = pci_alloc_irq_vectors(dev, 1, 1, PCI_IRQ_ALL_TYPES|PCI_IRQ_AFFINITY);
    if (rc < 0) {
        dev_err(&dev->dev, "Failed to claim the PCI interrupt\n");
        goto err;
    }
    irq = pci_irq_vector(dev, 0);
#else
    rc = pci_enable_msi(dev);
    if (rc < 0) {
        DPRINTK("no MSI interrupts, using legacy irq\n");
    }
    irq = dev->irq;
#endif
    DPRINTK("Using irq %d\n",irq);
    rc = devm_request_threaded_irq(&dev->dev, irq, minit_isr_quick,
                    minit_isr, IRQF_SHARED,
                    "minit", minit_dev);
    if (rc) {
        dev_err(&dev->dev, "failed to claim IRQ %d\n", dev->irq);
        goto err;
    }

    pci_set_master(dev);

    if (!pci_set_dma_mask(dev, DMA_BIT_MASK(64))) {
        DPRINTK("64-bit DMA mask\n");
    } else if (!pci_set_dma_mask(dev, DMA_BIT_MASK(32))) {
        DPRINTK("32-bit DMA mask\n");
    } else {
        dev_warn(&dev->dev, "No suitable DMA available\n");
        goto err;
    }

    if (!pci_set_consistent_dma_mask(dev, DMA_BIT_MASK(64))) {
        DPRINTK("64-bit DMA mask\n");
    } else if (!pci_set_consistent_dma_mask(dev, DMA_BIT_MASK(32))) {
        DPRINTK("32-bit DMA mask\n");
    } else {
        dev_warn(&dev->dev, "No suitable DMA available\n");
        goto err;
    }

    // create character device
    char_device = device_create(
        minit_class,
        &dev->dev,
        MKDEV(minit_major, minit_dev->minor_dev_no),
        minit_dev,
        "%s_%d", ONT_DRIVER_NAME, minit_dev->minor_dev_no);
    if (IS_ERR(char_device)) {
        dev_info(&dev->dev, "unable to create a sysfs device.\n");
        rc = PTR_ERR(char_device);
        goto err;
    }

    // register the Altera I2C core using a slightly modified Altera driver
    rc = borrowed_altr_i2c_probe(minit_dev);
    if (rc) {
        dev_err(&dev->dev, ": I2C bus controller probe failed\n");
        goto err;
    }

    // add to our internal device table
    rc = device_table_add(minit_dev->minor_dev_no, minit_dev);

    // enable interrupts
    WRITEL(PCI_ISR_I2C /*| PCI_ISR_DMA*/, minit_dev->pci_bar + PCI_ENB);

    // enable clock in ASIC control
    writeb(ASIC_CTRL_CLK_128, minit_dev->ctrl_bar + ASIC_CTRL_BASE);

    DPRINTK("probe finished successfully\n");
err:
    return rc;
}

static void __exit pci_remove(struct pci_dev *dev)
{
    DPRINTK("pci_remove called with %p\n",dev);
    if (!dev) {
        printk(KERN_ERR"pci_remove called with null\n");
        return;
    }
}

static int __init ont_minit1c_init(void)
{
    int rc = 0;
    dev_t first_dev;
    DPRINTK("ont_minit1c_init\n");

    // grab some character device numbers
    rc = alloc_chrdev_region(
        &first_dev,
        ONT_FIRST_MINOR,
        MINIT_MAX_DEVICES,
        ONT_DRIVER_NAME);
    minit_major = MAJOR(first_dev);
    minit_minor = MINOR(first_dev);
    if (rc < 0) {
        printk(KERN_ERR ONT_DRIVER_NAME": Failed to allocate major number\n");
        goto err_grab_chrdev;
    }

    // define a type of sysfs class associated with this module
    minit_class = class_create(THIS_MODULE, ONT_DRIVER_NAME);
    if (IS_ERR(minit_class)) {
        rc = PTR_ERR(minit_class);
        printk(KERN_ERR"Failed to create ont_minit1c device class\n");
        goto err_reg_class;
    }
    cdev_init(&minit_cdev, &minit_fops);
    rc = cdev_add(&minit_cdev, first_dev, MINIT_MAX_DEVICES);
    if (rc < 0) {
        printk(KERN_ERR"Failed to add cdev\n");
        goto err_reg_cdev;
    }

    // let the pci stack know we exist
    rc = pci_register_driver(&minit_driver_ops);
    if (rc) {
        printk(KERN_ERR ONT_DRIVER_NAME": PCI driver registration failed\n");
        goto err_reg_driver;
    }
    DPRINTK("Success\n");
    return rc;

err_reg_driver:
    cdev_del(&minit_cdev);
err_reg_cdev:
    class_destroy(minit_class);
err_reg_class:
    unregister_chrdev_region(first_dev, MINIT_MAX_DEVICES);
err_grab_chrdev:
    return rc;
}

static void __exit ont_minit1c_exit(void)
{
    pci_unregister_driver(&minit_driver_ops);
    cdev_del(&minit_cdev);
    if (minit_class) {
        class_destroy(minit_class);
    }
    unregister_chrdev_region(MKDEV(minit_major, ONT_FIRST_MINOR), MINIT_MAX_DEVICES);
}


module_init(ont_minit1c_init);
module_exit(ont_minit1c_exit);

MODULE_AUTHOR("Richard Crewe <richard.crewe@nanoporetech.com>");
MODULE_DESCRIPTION("ONT MinIT-1C PCIe Driver");
MODULE_VERSION(ONT_DRIVER_VERSION);

MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(pci, pci_ids);
