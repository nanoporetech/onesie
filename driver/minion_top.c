/**
 * minion_top.c
 *
 * Copyright (C) 2019 Oxford Nanopore Technologies Ltd.
 *
 * Author: <info@nanoporetech.com>
 *
 * This file contains register definitions for the MinION-1C firmware
 *
 */

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
#include <linux/bug.h>

#include "minion_top.h"
#include "minion_reg.h"
#include "minion_ioctl.h"

/*
 * PROTOTYPES
 */

static int __init pci_probe(struct pci_dev *dev, const struct pci_device_id *id);
static void __exit pci_remove(struct pci_dev *dev);
static int minion_file_open(struct inode* inode, struct file *file);
static int minion_file_close(struct inode *inode, struct file *file);
static long minion_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int switch_link_mode(struct minion_device_s* , const enum link_mode_e , struct historical_link_mode* );
static void free_link(struct minion_device_s* , const struct historical_link_mode* );
/*
 * STRUCTURES
 */
static struct pci_device_id pci_ids[] = {
    { PCI_DEVICE(0x1ab0, 0x0010) }, // Altera development card
    { PCI_DEVICE(0x1e59, 0x0001) }, // Nanopore Tech Hardware
    { 0 }
};

static struct pci_driver minion_driver_ops = {
    .name = ONT_DRIVER_NAME,
    .id_table = pci_ids,
    .probe = pci_probe,
    .remove = pci_remove,
};

static struct file_operations minion_fops =
{
    .owner          = THIS_MODULE,
    .open           = minion_file_open,
    .unlocked_ioctl = minion_unlocked_ioctl,
    .release        = minion_file_close,
};

/*
 * MODULE GLOBALS
 */
static struct class* minion_class = NULL;
static int minion_major = 0;
static int minion_minor = 0;
static struct cdev minion_cdev = {};

/* Array of minion_device pointers indexed by device-node minor number */
static struct minion_device_s* minion_device_table[MINION_MAX_DEVICES] = {};


/*
 * FUNCTIONS
 */

/**
 * @return <0 on error, 0 on OK
 */
static int device_table_add(unsigned int minor, struct minion_device_s* mdev)
{
    VPRINTK("device_table_add\n");
    if (minor >= MINION_MAX_DEVICES) {
        printk(KERN_ERR ONT_DRIVER_NAME":Too many devices\n");
        return -EINVAL;
    }

    if (minion_device_table[minor]) {
        printk(KERN_ERR ONT_DRIVER_NAME":That device node is already in use\n");
        return -EEXIST;
    }

    DPRINTK("Adding %p at %d\n", mdev, minor);
    minion_device_table[minor] = mdev;
    return 0;
}

/**
 * @return <0 on error, minor number of removed device will be returned if OK
 */
static int device_table_remove(struct minion_device_s* mdev)
{
    int index;
    VPRINTK("device_table_remove\n");
    for (index = 0; index < MINION_MAX_DEVICES; ++index) {
        if (minion_device_table[index] == mdev) {
            DPRINTK("Removing entry %p at %d\n",mdev, index);
            minion_device_table[index] = 0;
            return index;
        }
    }
    DPRINTK("%p not found in device table\n",mdev);
    return -ENOENT;
}

static struct minion_device_s* device_table_lookup(unsigned int minor)
{
    VPRINTK("device_table_lookup\n");
    if (minor >= MINION_MAX_DEVICES) {
        printk(KERN_ERR ONT_DRIVER_NAME":invalid device number %d\n",minor);
        return NULL;
    }

    return minion_device_table[minor];
}

/**
 * This performs the register reading and writing for IOCTLs. It bound checks
 * register offsets, but does not enforse alignment
 * @param mdev device in which to access the register
 * @param reg_access contains details of the register access and will be
 *     modified with read values
 * @return 0 on success
 */
static long minion_reg_access(struct minion_device_s* mdev, struct minion_register_s* reg_access)
{
    long rc = 0;
    void* address;

    BUILD_BUG_ON(sizeof(struct minion_register_s) != 16);
    if (unlikely(!mdev)) {
        printk(KERN_ERR ONT_DRIVER_NAME": ioctl called when there is no hardware.\n");
        return -ENODEV;
    }

    /* select bar and check for out of bounds accesses */
    switch (reg_access->bar) {
    case CTRL_BAR:
        address = mdev->ctrl_bar;
        if ((reg_access->offset + reg_access->size) > CTRL_BAR_EXPECTED_SIZE) {
            return -EFAULT;
        }
        break;
    case SPI_BAR:
        address = mdev->spi_bar;
        if ((reg_access->offset + reg_access->size) > SPI_BAR_EXPECTED_SIZE) {
            return -EFAULT;
        }
        break;
    case PCI_BAR:
        address = mdev->pci_bar;
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
            reg_access->value = (u64)readl(address);
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
 * @param mdev pointer do driver structure
 * @param to_dev 282 byte buffer to send to ASIC
 * @param from_dev 282 byte buffer to receive data from ASIC
 * @param start  start transfer
 * @param enable enable module
 * @param clk clockspeed in Hz, this will be achieved by integer division of
 * the 62.5 MHz PCIe clock
 * @param cmd_id command-id
 * @return
 */
static long minion_shift_register_access(
        struct minion_device_s* mdev,
        char* const to_dev,
        char* const from_dev,
        const u8 start,
        const u8 enable,
        const u32 clk,
        u8* cmd_id)
{
    struct historical_link_mode old_link_mode;
    long rc;
    u32 clockdiv;
    u32 actual_clock;
    u32 control;
    unsigned int delay_ms = 1+((1000 * ASIC_SHIFT_REG_SIZE) / clk);
    VPRINTK("minion_shift_register_access to_dev %p, from_dev %p, start %d, enable %d, clk %d\n",
            to_dev, from_dev, start, enable, clk);

    rc = switch_link_mode(mdev, link_mode_data, &old_link_mode);
    if (rc < 0) {
        return rc;
    }

    // write to data into shift register
    if (to_dev) {
        int i;
        for (i = 0; i < ASIC_SHIFT_REG_SIZE; ++i) {
            VPRINTK("Shift to dev  : 0x%02x => %p\n",from_dev[i],mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_OUTPUT_BUF + i);
            writeb(to_dev[i], mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_OUTPUT_BUF + i);
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
    VPRINTK("shift reg control 0x%02x => %p\n", control, mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_CTRL);
    writeb(control, mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_CTRL);
    wmb();

    if (to_dev) {
        writeb(*cmd_id, mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_CMD_ID);
    }

    if (from_dev) {
        int i;
        // wait for the data to move
        VPRINTK("sleeping for %d ms\n",delay_ms);
        msleep(delay_ms);

        for (i = 0; i < ASIC_SHIFT_REG_SIZE; ++i) {
            from_dev[i] = readb(mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_INPUT_BUF + i);
            VPRINTK("Shift from dev: 0x%02x <= %p\n",from_dev[i],mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_INPUT_BUF + i);
        }
        *cmd_id = readb(mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_CMD_ID);
    }

    free_link(mdev, &old_link_mode);

    return 0;
}

static void minion_hs_reg_access(struct minion_device_s* mdev, struct minion_hs_receiver_s* minion_hs_reg)
{
    unsigned int i;


    DPRINTK("minion_hs_reg_access\n");
    if (minion_hs_reg->write) {
        for (i = 0; i < NUM_HS_REGISTERS;++i) {
            if (((ASIC_HS_REG_WRITE_MASK >> i) & 1) == 1) {
                writew(minion_hs_reg->registers[i], mdev->ctrl_bar + ASIC_HS_RECEIVER_BASE + (i * 2));
            }
        }

    }

    // now read registers
    for (i = 0; i < NUM_HS_REGISTERS;++i) {
        minion_hs_reg->registers[i] = readw(mdev->ctrl_bar + ASIC_HS_RECEIVER_BASE + (i * 2));
    }
}


/*
 * want to lock-out other modes until done, but allow re-entrance for things running the same mode
 */
static int switch_link_mode(struct minion_device_s* mdev, const enum link_mode_e mode, struct historical_link_mode* old_mode)
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

static void free_link(struct minion_device_s* mdev, const struct historical_link_mode* old_mode)
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
 * @param mdev minion device
 * @param buffer pointer to the buffer for the data
 * @param start  start index in eeprom
 * @param length number of bytes to read
 * @return error if negative, 0 if OK
 */
static long read_eeprom(struct minion_device_s* mdev, u8* buffer, u32 start, u32 length)
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
 * @param mdev minioon device
 * @param buffer pointer to the buffer containing the data
 * @param start  start index in eeprom
 * @param length number of bytes to read
 * @return error if negative, 0 if OK
 */
static long write_eeprom(struct minion_device_s* mdev, u8* buffer, u32 start, u32 length)
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
static int minion_file_open(struct inode* inode, struct file *file)
{
    struct minion_device_s* mdev;

    VPRINTK("minion_file_open\n");
    /* work out which hardware the user is expecting to talk to from the device no */
    mdev = device_table_lookup(iminor(inode));
    if (!mdev) {
        printk(KERN_ERR ONT_DRIVER_NAME":No MINION device with minor device number %d\n",iminor(inode));
        return -ENODEV;
    }

    file->private_data = mdev;
    return 0;
}

/**
 * @TODO: fill this in
 */
static int minion_file_close(struct inode *inode, struct file *file)
{
    struct minion_device_s* mdev;

    VPRINTK("minion_file_close\n");
    mdev = (struct minion_device_s*)file->private_data;
    if (!mdev) {
        DPRINTK("file->private_data null, can't clear-up!\n");
        return 0;
    }

    // cancel all transfers
    cancel_data_transfers(mdev->dma_dev);

    return 0;
}

static long minion_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long rc = 0;
    struct minion_device_s* mdev = file->private_data;

    VPRINTK("IOCTL file %p cmd %u, arg %lu\n",file, cmd, arg);
    VPRINTK("mdev = %p\n",mdev);
    if (_IOC_TYPE(cmd) != 'b') {
        DPRINTK("bad magic number\n");
        return -ENOTTY;
    }

    switch(cmd) {
    case MINION_IOCTL_REG_ACCESS: {
            struct minion_register_s reg_access = {};
            BUILD_BUG_ON(sizeof(struct minion_register_s) != MINION_REGISTER_SIZE);
            VPRINTK("MINION_IOCTL_REG_ACCESS\n");
            rc = copy_from_user(&reg_access, (void __user*)arg, sizeof(struct minion_register_s) );
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
            }
            rc = minion_reg_access(mdev, &reg_access);
            if (rc) {
                DPRINTK("minion_reg_access failed\n");
                return rc;
            }
            return copy_to_user((void __user*)arg, &reg_access, sizeof(struct minion_register_s) );
        }
        break;
    case MINION_IOCTL_SHIFT_REG: {
            struct minion_shift_reg_s shift_reg_access = {};
            char shift_reg[ASIC_SHIFT_REG_SIZE];
            BUILD_BUG_ON(sizeof(struct minion_shift_reg_s) != MINION_SHIFT_REG_SIZE);
            VPRINTK("MINION_IOCTL_SHIFT_REG\n");
            rc = copy_from_user(&shift_reg_access, (void __user*)arg, sizeof(struct minion_shift_reg_s));
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
            }
            if (shift_reg_access.padding) {
                dev_err(&mdev->pci_device->dev, "Padding in IOCTL not zero");
                return -EINVAL;
            }
            if (shift_reg_access.to_device) {
                rc = copy_from_user(shift_reg, (void __user*)shift_reg_access.to_device, ASIC_SHIFT_REG_SIZE);
                if (rc) {
                    DPRINTK("copy_from_user failed\n");
                    return rc;
                }
            }

            // do access
            rc = minion_shift_register_access(
                        mdev,
                        shift_reg_access.to_device ? shift_reg : NULL,
                        shift_reg_access.from_device ? shift_reg : NULL,
                        shift_reg_access.start,
                        shift_reg_access.enable,
                        shift_reg_access.clock_hz,
                        &shift_reg_access.command_id);
            if (rc) {
                DPRINTK("shift register operation failed\n");
                return rc;
            }
            if (shift_reg_access.from_device) {
                rc = copy_to_user((void __user*)shift_reg_access.from_device, shift_reg, ASIC_SHIFT_REG_SIZE);
                if (rc) {
                    DPRINTK("copy_to_user failed\n");
                    return rc;
                }
            }
            return copy_to_user((void __user*)arg, &shift_reg_access, sizeof(struct minion_shift_reg_s) );
        }
        break;
    case MINION_IOCTL_HS_RECIEVER: {
            struct minion_hs_receiver_s minion_hs_reg = {};
            BUILD_BUG_ON(sizeof(struct minion_hs_receiver_s) != MINION_HS_RECEIVER_SIZE);
            VPRINTK("MINION_IOCTL_HS_RECIEVER\n");
            rc = copy_from_user(&minion_hs_reg, (void __user*)arg, sizeof(minion_hs_reg));
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
            }
            // check padding is zero
            if (minion_hs_reg.padding[0] ||
                minion_hs_reg.padding[1] ||
                minion_hs_reg.padding[2] )
            {
                dev_err(&mdev->pci_device->dev, "Padding in IOCTL not zero");
                return -EINVAL;
            }
            minion_hs_reg_access(mdev, &minion_hs_reg);

            return copy_to_user((void __user*) arg, &minion_hs_reg, sizeof(minion_hs_reg));
        }
        break;
    case MINION_IOCTL_EEPROM_READ: {
            struct minion_eeprom_transfer_s transfer;
            u8 k_buffer[EEPROM_SIZE] = {0};
            BUILD_BUG_ON(sizeof(struct minion_eeprom_transfer_s) != MINION_EEPROM_TRANSFER_SIZE);
            VPRINTK("MINION_IOCTL_EEPROM_READ\n");
            rc = copy_from_user(&transfer, (void __user*)arg, sizeof(transfer));
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
            }
            rc = read_eeprom(mdev, k_buffer, transfer.start, transfer.length );
            if (rc) {
                DPRINTK("eeprom access failed\n");
                return rc;
            }
            // transfer.length is assumed to have been checked for a safe size
            // in read_eeprom
            rc = copy_to_user((void __user*)transfer.data, k_buffer, transfer.length);
            if (rc) {
                DPRINTK("failed to copy eeprom back into userspace\n");
                return rc;
            }
            return 0;
        }
        break;
    case MINION_IOCTL_EEPROM_WRITE: {
            struct minion_eeprom_transfer_s transfer;
            u8 k_buffer[EEPROM_SIZE] = {0};
            BUILD_BUG_ON(sizeof(struct minion_eeprom_transfer_s) != MINION_EEPROM_TRANSFER_SIZE);
            VPRINTK("MINION_IOCTL_EEPROM_WRITE\n");
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
            rc = copy_from_user(k_buffer, (void __user*)transfer.data, transfer.length);
            if (rc) {
                DPRINTK("copy_from_user failed to get data from user-space\n");
                return rc;
            }
            rc = write_eeprom(mdev, k_buffer, transfer.start, transfer.length );
            if (rc) {
                DPRINTK("eeprom access failed\n");
                return rc;
            }
            return 0;
        }
        break;
    case MINION_IOCTL_SUBMIT_TRANSFER: {
            struct minion_data_transfer_s transfer;
            BUILD_BUG_ON(sizeof(struct minion_data_transfer_s) != MINION_DATA_TRANSFER_SIZE);
            VPRINTK("MINION_IOCTL_SUBMIT_TRANSFER\n");
            rc = copy_from_user(&transfer, (void __user*)arg, sizeof(transfer));
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
            }
            // make a transfer object to represent the transfer when in the driver
            rc = queue_data_transfer(mdev->dma_dev, &transfer, file);
            return rc;
        }
        break;
    case MINION_IOCTL_WHATS_COMPLETED: {
            struct minion_completed_transfers_s completed;
            struct minion_transfer_status_s* stuff_done;
            BUILD_BUG_ON(sizeof(struct minion_transfer_status_s) != MINION_TRANSFER_STATUS_SIZE);
            BUILD_BUG_ON(sizeof(struct minion_completed_transfers_s) != MINION_COMPLETED_TRANSFERS_SIZE);
            VPRINTK("MINION_IOCTL_WHATS_COMPLETED\n");
            rc = copy_from_user(&completed, (void __user*)arg, sizeof(completed));
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
            }

            // Allocate memory to prepare the list of completed transfers.
            /// @TODO: This could possibly be done by mapping the user-space buffer into k-space.
            stuff_done = kzalloc(sizeof(struct minion_transfer_status_s) * completed.completed_transfers_size, GFP_KERNEL);
            if (!stuff_done) {
                DPRINTK("Failed to allocate memory for completed transfer list\n");
                return -ENOMEM;
            }
            completed.no_completed_transfers = get_completed_data_transfers(
                        mdev->dma_dev,
                        completed.completed_transfers_size,
                        stuff_done );
            rc = copy_to_user(
                        (void __user*)completed.completed_transfers,
                        stuff_done,
                        sizeof(struct minion_transfer_status_s) * completed.completed_transfers_size );
            if (!rc) {
                rc = copy_to_user(
                        (void __user*)arg,
                        &completed,
                        sizeof(completed) );
            }
            kfree(stuff_done);
            if (rc) {
                DPRINTK("copy_to_user failed\n");
                return rc;
            }
            return 0;
        }
        break;
    case MINION_IOCTL_CANCEL_TRANSFERS: {
            VPRINTK("MINION_IOCTL_CANCEL_TRANSFERS\n");
            return cancel_data_transfers(mdev->dma_dev);
        }
        break;
    default:
        printk(KERN_ERR ONT_DRIVER_NAME": Invalid ioctl for this device (%u)\n", cmd);
    }
    return -EIO;
}


static void cleanup_device(void* data) {
    struct pci_dev* dev = (struct pci_dev*)data;
    struct minion_device_s* mdev = pci_get_drvdata(dev);
    VPRINTK("cleanup_device\n");
    if (mdev) {
        device_table_remove(mdev);
        device_destroy(minion_class, MKDEV(minion_major, mdev->minor_dev_no));
    }
}

static irqreturn_t minion_isr_quick(int irq, void* _dev)
{
    u32 isr;
    struct minion_device_s* mdev = _dev;
    irqreturn_t ret_i2c = IRQ_NONE;
    irqreturn_t ret_dma = IRQ_NONE;

    VPRINTK("minion_isr_quick\n");
    isr = READL(mdev->pci_bar + PCI_ISR);
    if (!isr) {
        return IRQ_NONE;
    }
    WRITEL(isr, mdev->pci_bar + PCI_ISR);

    // call the quick ISR for the two cores that can generate interrupts
    if (mdev->i2c_isr_quick ) {
        ret_i2c = mdev->i2c_isr_quick(irq, mdev->i2c_dev);
        if (ret_i2c == IRQ_WAKE_THREAD) {
            set_bit(0, &mdev->had_i2c_irq);
        }
    }

    if (mdev->dma_isr_quick ) {
        ret_dma = mdev->dma_isr_quick(irq, mdev->dma_dev);
        if (ret_dma == IRQ_WAKE_THREAD) {
            set_bit(0, &mdev->had_dma_irq);
        }
    }

    // either wake a bh thread, mark the irq as handled or not-us depending
    // on what the above discovered.
    if (ret_i2c == IRQ_WAKE_THREAD || ret_dma == IRQ_WAKE_THREAD) {
        VPRINTK("minion_isr_quick wake\n");
        return IRQ_WAKE_THREAD;
    }
    if (ret_i2c == IRQ_HANDLED || ret_dma == IRQ_HANDLED) {
        VPRINTK("minion_isr_quick handled\n");
        return IRQ_HANDLED;
    }
    VPRINTK("minion_isr_quick none\n");
    return IRQ_NONE;
}

static irqreturn_t minion_isr(int irq, void* _dev)
{
    struct minion_device_s* mdev = _dev;
    irqreturn_t ret_i2c = IRQ_NONE;
    irqreturn_t ret_dma = IRQ_NONE;

    VPRINTK("minion_isr (bh)\n");
    // should only get an IRQ from I2C core if it has a message
    if (mdev->i2c_isr && test_and_clear_bit(0,&mdev->had_i2c_irq)) {
        ret_i2c = mdev->i2c_isr(irq, mdev->i2c_dev);
    }
    if (mdev->dma_isr && test_and_clear_bit(0,&mdev->had_dma_irq)) {
        ret_dma = mdev->dma_isr(irq, mdev->dma_dev);
    }

    if (ret_i2c == IRQ_HANDLED || ret_dma == IRQ_HANDLED) {
        VPRINTK("minion_isr handled\n");
        return IRQ_HANDLED;
    }
    VPRINTK("minion_isr none\n");
    return IRQ_NONE;
}

/**
 * This should:
 *  Verify that we're talking to usable minion hardware.
 *  Map bars and create the device data structure.
 *
 */
static int __init pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    int rc,irq;
    struct minion_device_s* mdev = NULL;
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
    mdev = devm_kzalloc(&dev->dev, sizeof(struct minion_device_s), GFP_KERNEL);
    if (!mdev) {
        dev_err(&dev->dev, "Unable to allocate memory for managing device\n");
        goto err;
    }
    mdev->link_mode = link_idle;
    mutex_init(&mdev->link_mtx);
    mdev->pci_device = dev;
    mdev->minor_dev_no = minion_minor++;
    pci_set_drvdata(dev, mdev);

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

    mdev->ctrl_bar  = pcim_iomap_table(dev)[CTRL_BAR];
    mdev->spi_bar = pcim_iomap_table(dev)[SPI_BAR];
    mdev->pci_bar = pcim_iomap_table(dev)[PCI_BAR];
    DPRINTK("Control bar mapped to %p\n", mdev->ctrl_bar);
    DPRINTK("SPI bar mapped to %p\n", mdev->spi_bar);
    DPRINTK("PCI bar mapped to %p\n", mdev->pci_bar);

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
    rc = devm_request_threaded_irq(&dev->dev, irq, minion_isr_quick,
                    minion_isr, IRQF_SHARED,
                    "minion", mdev);
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
        minion_class,
        &dev->dev,
        MKDEV(minion_major, mdev->minor_dev_no),
        mdev,
        "%s_%d", ONT_DRIVER_NAME, mdev->minor_dev_no);
    if (IS_ERR(char_device)) {
        dev_info(&dev->dev, "unable to create a sysfs device.\n");
        rc = PTR_ERR(char_device);
        goto err;
    }

    // register the Altera I2C core using a slightly modified Altera driver
    rc = borrowed_altr_i2c_probe(mdev);
    if (rc) {
        dev_err(&dev->dev, ": I2C bus controller probe failed\n");
        goto err;
    }
    devm_add_action(&dev->dev, borrowed_altr_i2c_remove, mdev);

    rc = altera_sgdma_probe(mdev);
    if (rc) {
        dev_err(&dev->dev, ": DMA hardware probe failed\n");
        goto err;
    }
    devm_add_action(&dev->dev, altera_sgdma_remove, mdev);

    // add to our internal device table
    rc = device_table_add(mdev->minor_dev_no, mdev);

    // enable interrupts
    WRITEL(PCI_ISR_I2C | PCI_ISR_DMA, mdev->pci_bar + PCI_ENB);

    // enable clock in ASIC control
    writeb(ASIC_CTRL_CLK_128, mdev->ctrl_bar + ASIC_CTRL_BASE);

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

static int __init minion_init(void)
{
    int rc = 0;
    dev_t first_dev;
    DPRINTK("minion_init\n");

    // grab some character device numbers
    rc = alloc_chrdev_region(
        &first_dev,
        ONT_FIRST_MINOR,
        MINION_MAX_DEVICES,
        ONT_DRIVER_NAME);
    minion_major = MAJOR(first_dev);
    minion_minor = MINOR(first_dev);
    if (rc < 0) {
        printk(KERN_ERR ONT_DRIVER_NAME": Failed to allocate major number\n");
        goto err_grab_chrdev;
    }

    // define a type of sysfs class associated with this module
    minion_class = class_create(THIS_MODULE, ONT_DRIVER_NAME);
    if (IS_ERR(minion_class)) {
        rc = PTR_ERR(minion_class);
        printk(KERN_ERR"Failed to create minion device class\n");
        goto err_reg_class;
    }
    cdev_init(&minion_cdev, &minion_fops);
    rc = cdev_add(&minion_cdev, first_dev, MINION_MAX_DEVICES);
    if (rc < 0) {
        printk(KERN_ERR"Failed to add cdev\n");
        goto err_reg_cdev;
    }

    // let the pci stack know we exist
    rc = pci_register_driver(&minion_driver_ops);
    if (rc) {
        printk(KERN_ERR ONT_DRIVER_NAME": PCI driver registration failed\n");
        goto err_reg_driver;
    }
    DPRINTK("Success\n");
    return rc;

err_reg_driver:
    cdev_del(&minion_cdev);
err_reg_cdev:
    class_destroy(minion_class);
err_reg_class:
    unregister_chrdev_region(first_dev, MINION_MAX_DEVICES);
err_grab_chrdev:
    return rc;
}

static void __exit minion_exit(void)
{
    pci_unregister_driver(&minion_driver_ops);
    cdev_del(&minion_cdev);
    if (minion_class) {
        class_destroy(minion_class);
    }
    unregister_chrdev_region(MKDEV(minion_major, ONT_FIRST_MINOR), MINION_MAX_DEVICES);
}


module_init(minion_init);
module_exit(minion_exit);

MODULE_AUTHOR("Oxford Nanopore Technologies Ltd <info@nanoporetech.com>");
MODULE_DESCRIPTION("MinION-mk1C PCIe Driver");
MODULE_VERSION(ONT_DRIVER_VERSION);

MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(pci, pci_ids);
