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
#include <linux/sysfs.h>

#include "minion_top.h"
#include "minion_reg.h"
#include "minion_ioctl.h"
#include "thermal_interface.h"

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
 * @brief calculate shift-register core clock-divider
 * @param clk desired clock-speed in Hz
 */
static u32 calculate_shift_reg_clock_divider(const u32 clk)
{
    u32 clockdiv;
    u32 actual_clock;
    if (clk > ASIC_SHIFT_MAX_CLOCK) {
        clockdiv = 0;
    } else if (clk < ASIC_SHIFT_MIN_CLOCK) {
        clockdiv = ASIC_SHIFT_CTRL_DIV_MAX;
    } else {
        clockdiv = ((PCIe_LANE_CLOCK/(2*clk)) - 1) / 2;
    }
    actual_clock = ASIC_SHIFT_DIV_TO_CLOCK(clockdiv);
    if (actual_clock != clk) {
        DPRINTK("Requested SPI Clock of %d couldn't be achieved, best effort %d\n",
                clk, actual_clock);
    }
    return clockdiv;
}

/**
 * @brief write the shift-register, wavetable and wavetable-control registers
 * @param mdev pointer to driver structure
 * @param param parameters containing shift-register and optional wavetable data
 */
static void write_shift_reg_hw(
        struct minion_device_s* mdev,
        struct shift_reg_access_parameters_s* param)
{
    u16 tmp;
    unsigned int i;

    if (!param->to_dev) {
        return;
    }

    for (i = 0; i < ASIC_SHIFT_REG_SIZE; i += 2) {
        VPRINTK("Shift to dev  : 0x%02x => [%p]\n", param->to_dev[i], mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_OUTPUT_BUF + i);
        VPRINTK("Shift to dev  : 0x%02x => [%p]\n", param->to_dev[i+1], mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_OUTPUT_BUF + i + 1);
        tmp = (param->to_dev[i+1] << 8u) | param->to_dev[i];
        writew(tmp, mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_OUTPUT_BUF + i);
    }

    if (param->wavetable) {
        for (i = 0; i < MINION_WAVEFORM_SIZE; ++i) {
            writew(param->wavetable[i], mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_WAVE_TABLE + (i*2));
        }
    }

    // waveform controls. The length and frames components of these registers
    // hold "value - 1" so a register-value of 0 encodes a length or frame-count
    // of 1. waveform-frames controls if the waveform is enabled with non-zero
    // values enabing the bias-voltage waveform.
    tmp = ((param->waveform_length-1) & ASIC_SHIFT_LUT_LEN_MASK) |
          (param->waveform_frames ? ASIC_SHIFT_LUT_ENABLE : 0);
    writew(tmp, mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_WAVE_CTRL1 );

    tmp = ((param->waveform_frames-1) & ASIC_SHIFT_WAVE_FRAMES_MASK);
    writew(tmp, mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_WAVE_CTRL2 );
}

/**
 * @brief read the shift-register, wavetable, wavetable-control and asic-config-id registers
 * @param mdev pointer to driver structure
 * @param param parameters with space for the read shift-register values,
 *              asic-configuration id and optional wave-table.
 */
static void read_shift_reg_hw(
        struct minion_device_s* mdev,
        struct shift_reg_access_parameters_s* param)
{
    int i;
    u16 tmp;
    unsigned int delay_ms = 1+((1000 * ASIC_SHIFT_REG_SIZE) / param->clk);

    if (!param->from_dev) {
        return;
    }

    // wait for the data to move
    VPRINTK("sleeping for %d ms\n",delay_ms);
    msleep(delay_ms);

    for (i = 0; i < ASIC_SHIFT_REG_SIZE; i += 2) {
        tmp = readw(mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_INPUT_BUF + i);
        param->from_dev[i] = tmp & 0xff;
        param->from_dev[i+1] = tmp >> 8;
        VPRINTK("Shift from dev: 0x%02x <= [%p]\n", param->from_dev[i], mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_INPUT_BUF + i);
        VPRINTK("Shift from dev: 0x%02x <= [%p]\n", param->from_dev[i+1], mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_INPUT_BUF + i + 1);
    }

    if (param->wavetable) {
        for (i = 0; i < MINION_WAVEFORM_SIZE; ++i) {
            param->wavetable[i] = readw(mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_WAVE_TABLE + (i*2));
        }
    }

    // waveform controls. The length and frames components of these registers
    // hold "value - 1" so a register-value of 0 encodes a length or frame-count
    // of 1
    tmp = readw(mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_WAVE_CTRL1);
    param->waveform_length = (tmp & ASIC_SHIFT_LUT_LEN_MASK) + 1;

    tmp = readw(mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_WAVE_CTRL2);
    // waveform frame-count encodes if the waveform is enabled.
    param->waveform_frames  = (tmp & ASIC_SHIFT_LUT_ENABLE) ? (tmp & ASIC_SHIFT_WAVE_FRAMES_MASK) + 1 : 0;

    // read command-id / ASIC-config id
    param->cmd_id = readw(mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_CTRL) >> ASIC_SHIFT_CTRL_CMDID_SHIFT;
}

/**
 * @brief data transfer and control via shift-register
 *
 * This carries out some of the user-space to kernel-space data movement for
 * the shift_register_access IOCTL and converts the parameters from their
 * external to internal abstractions.
 *
 * @param mdev pointer to driver structure
 * @param param parameters to/from shift-reg core registers
 * @return an ioctl return code
 */
static long minion_shift_register_access(
        struct minion_device_s* mdev,
        struct shift_reg_access_parameters_s* param)
{
    struct historical_link_mode old_link_mode;
    long rc;
    u32 clockdiv;
    u16 control;
    VPRINTK("minion_shift_register_access to_dev %p, from_dev %p, start %d, enable %d, clk %d\n",
            param->to_dev,param->from_dev, param->start, param->enable, param->clk);

    rc = switch_link_mode(mdev, link_mode_data, &old_link_mode);
    if (rc < 0) {
        return rc;
    }

    // write to data into shift register
    write_shift_reg_hw(mdev, param);

    wmb();

    clockdiv = calculate_shift_reg_clock_divider(param->clk);

    control = (clockdiv << ASIC_SHIFT_CTRL_DIV_SHIFT) |
              (param->start ? ASIC_SHIFT_CTRL_ST : 0) |
              (param->enable ? ASIC_SHIFT_CTRL_EN :0) |
              (param->cmd_id << ASIC_SHIFT_CTRL_CMDID_SHIFT);
    VPRINTK("shift reg control 0x%02x => %p\n", control, mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_CTRL);
    writew(control, mdev->ctrl_bar + ASIC_SHIFT_BASE + ASIC_SHIFT_CTRL);
    wmb();

    read_shift_reg_hw(mdev, param);

    free_link(mdev, &old_link_mode);

    return 0;
}

/**
 * @brief shift-register core access wrapper
 *
 * This carries out some of the user-space to kernel-space data movement for
 * the shift_register_access IOCTL and converts the parameters from their
 * external to internal abstractions before calling minion_shift_register_access
 * to perform the hardware access.
 *
 * @param mdev pointer to driver structure
 * @param shift_reg_access shift-register core access parameters (in
 * kernel-space, but some members are pointers to user-space data.)
 * @return an ioctl return code
 */
static long minion_shift_reg_access_wrapper(struct minion_device_s* mdev, struct minion_shift_reg_s* shift_reg_access)
{
    long rc;
    char shift_reg[ASIC_SHIFT_REG_SIZE];
    u16* wavetable = kzalloc(MINION_WAVEFORM_SIZE * sizeof(u16), GFP_KERNEL);
    struct shift_reg_access_parameters_s parameters = {
        .to_dev   = shift_reg_access->to_device ? shift_reg : NULL,
        .from_dev = shift_reg_access->from_device ? shift_reg : NULL,
        .wavetable = shift_reg_access->waveform_table ? wavetable : NULL,
        .waveform_length = shift_reg_access->waveform_table_length,
        .waveform_frames = shift_reg_access->waveform_frame_count,
        .start = shift_reg_access->start,
        .enable = shift_reg_access->enable,
        .clk = shift_reg_access->clock_hz,
        .cmd_id = shift_reg_access->command_id,
    };

    if (!wavetable) {
        rc = -ENOMEM;
        goto err_out;
    }

    if (shift_reg_access->to_device) {
        rc = copy_from_user(shift_reg, (void __user*)shift_reg_access->to_device, ASIC_SHIFT_REG_SIZE);
        if (rc) {
            DPRINTK("copy_from_user failed\n");
            goto err_out;
        }
        if (shift_reg_access->waveform_table) {
            rc = copy_from_user(
                wavetable,
                (void __user*)shift_reg_access->waveform_table,
                MINION_WAVEFORM_SIZE * sizeof(u16));
            if (rc) {
                DPRINTK("copy_from_user failed for wavetable\n");
                goto err_out;
            }
        }
    }

    // do access
    rc = minion_shift_register_access(mdev,&parameters);
    if (rc) {
        DPRINTK("shift register operation failed\n");
        goto err_out;
    }

    // recover output parameters
    shift_reg_access->command_id = parameters.cmd_id;
    shift_reg_access->waveform_frame_count = parameters.waveform_frames;
    shift_reg_access->waveform_table_length = parameters.waveform_length;
    if (shift_reg_access->from_device) {
        rc = copy_to_user((void __user*)shift_reg_access->from_device, shift_reg, ASIC_SHIFT_REG_SIZE);
        if (rc) {
            DPRINTK("copy_to_user failed\n");
            goto err_out;
        }
        if (shift_reg_access->waveform_table) {
            rc = copy_to_user(
                (void __user*)shift_reg_access->waveform_table,
                wavetable,
                MINION_WAVEFORM_SIZE * sizeof(u16));
            if (rc) {
                DPRINTK("copy_from_user failed for wavetable\n");
                goto err_out;
            }
        }
    }
err_out:
    kfree(wavetable);
    return rc;
}

static void minion_hs_reg_access(struct minion_device_s* mdev, struct minion_hs_receiver_s* minion_hs_reg)
{
    unsigned int i;

    VPRINTK("minion_hs_reg_access\n");
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


static void get_fw_info(struct minion_device_s* mdev,
                        struct minion_firmware_info_s* fw_info)
{
    u32 version = readl(mdev->ctrl_bar + SYSTEM_ID_CORE + SYS_ID_VERSION);
    fw_info->major = version >> SYS_ID_VERSION_MAJOR_S;
    fw_info->minor = version >> SYS_ID_VERSION_MINOR_S;
    fw_info->patch = version;

    fw_info->timestamp = readl(mdev->ctrl_bar + SYSTEM_ID_CORE + SYS_ID_TIMESTAMP);
}

static void dump_firmware_info(struct minion_firmware_info_s* fw_info)
{
    printk(KERN_INFO"MinION-mk1C firmware version %d.%d.%d timestamp %u\n",
           fw_info->major,
           fw_info->minor,
           fw_info->patch,
           fw_info->timestamp);
}

/**
 * Convert from the temperatures used by the NIOS firmware to 8.8 fixed point
 */
static inline u16 temperature_to_fixedpoint(u16 temp)
{
    // limit to min 0C
    temp = max(temp, MIN_TEMPERATURE);
    return (temp / 2) - 4096;
}

/**
 * Convert to the temperatures used by the NIOS firmware from 8.8 fixed point
 */
static inline u16 fixedpoint_to_temperature(u16 fixed)
{
    // limit to a maximum of 1-bit under 112C
    fixed = min(fixed, MAX_TEMPERATURE);
    return 2 * (fixed + 4096);
}

static long apply_temp_cmd(struct minion_device_s* mdev, struct minion_temperature_command_s* tmp_cmd, bool write)
{
    long rc = 0;
    u16 latest_data_log_index = 0;
    u16* binary_command = NULL;
    u16* nios_message_ram_base = mdev->ctrl_bar + NIOS_MESSAGE_RAM_BASE;
    struct message_struct* temp_message = (struct message_struct*)nios_message_ram_base;

    // limit desired temperature to range [0C,50C]
    if (write && tmp_cmd->desired_temperature > MAX_SET_POINT) {
        rc = -EINVAL;
        goto exit;
    }

    // if required, check and allocate space for command
    if (tmp_cmd->binary_length > 0) {
        unsigned int length_words;
        unsigned int i;
        if (tmp_cmd->binary_length > sizeof(struct message_struct)) {
            rc = -EINVAL;
            goto exit;
        }

        // allocate kernel-space memory for the binary temperature command
        binary_command = kzalloc(tmp_cmd->binary_length, GFP_KERNEL);
        if (!binary_command) {
            rc = -ENOMEM;
            goto exit;
        }

        // copy the binary command from userspace if writing
        if (write) {
            rc = copy_from_user(binary_command, (void __user*)tmp_cmd->binary_data_pointer, tmp_cmd->binary_length);
            if (rc < 0) {
                goto exit;
            }
        }

        // read or write the command
        length_words = tmp_cmd->binary_length / sizeof(u16);
        for (i=0; i < length_words; ++i) {
            if (write) {
                writew(binary_command[i], nios_message_ram_base + i);
            } else {
                binary_command[i] = readw(nios_message_ram_base + i);
            }
        }

        // copy the binary command to userspace if reading
        if (!write) {
            rc = copy_to_user((void __user*)tmp_cmd->binary_data_pointer, binary_command, tmp_cmd->binary_length);
            if (rc < 0) {
                goto exit;
            }
        }
    }

    // set temperatures and control-word
    if (write) {
        if (tmp_cmd->control_word & CTRL_EN_MASK) {
            // if temperature control is enabled, write the temperature first, then control
            writew(fixedpoint_to_temperature(tmp_cmd->desired_temperature), &temp_message->set_point);
            writew(tmp_cmd->control_word, &temp_message->control_word);
        } else {
            // otherwise (if disabling temperature-control), write the control first, the temperature
            writew(tmp_cmd->control_word, &temp_message->control_word);
            writew(fixedpoint_to_temperature(tmp_cmd->desired_temperature), &temp_message->set_point);
        }
    }

    // read control, error and temperatures
    tmp_cmd->control_word = readw(&temp_message->control_word);
    tmp_cmd->error_word = readw(&temp_message->error_word);

    latest_data_log_index = min(readw(&temp_message->data_log_pointer), (u16)LOG_LEN);

    tmp_cmd->desired_temperature = temperature_to_fixedpoint(readw(&temp_message->set_point));
    tmp_cmd->flowcell_temperature = temperature_to_fixedpoint(
                readw(&temp_message->data_log[latest_data_log_index].fc_temp));
    tmp_cmd->heatsink_temperature = temperature_to_fixedpoint(
                readw(&temp_message->data_log[latest_data_log_index].hsink_temp));

exit:
    kfree(binary_command);

    return rc;
}

/**
 * disable temperature-control when the driver shuts down
 */
static void disable_temperature_control(void* ptr)
{
    struct minion_device_s* mdev = (struct minion_device_s*)ptr;
    u16* nios_message_ram_base = mdev->ctrl_bar + NIOS_MESSAGE_RAM_BASE;
    struct message_struct* temp_message = (struct message_struct*)nios_message_ram_base;

    writew(0, &temp_message->control_word);
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
            rc = minion_shift_reg_access_wrapper(mdev, &shift_reg_access);
            if (rc) {
                DPRINTK("copy_from_user failed\n");
                return rc;
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
            if (minion_hs_reg.padding[0] )
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
    case MINION_IOCTL_FIRMWARE_INFO: {
            struct minion_firmware_info_s fw_info;
            VPRINTK("MINON_IOCTL_FIRMWARE_INFO\n");
            get_fw_info(mdev,&fw_info);
            return copy_to_user((void __user*)arg, &fw_info, sizeof(fw_info));
        }
        break;
    case MINION_IOCTL_TEMP_CMD_READ:
    case MINION_IOCTL_TEMP_CMD_WRITE: {
            struct minion_temperature_command_s temp_cmd;
            VPRINTK("READ/WRITE TEMPERATURE_COMMAND\n");
            rc = copy_from_user(&temp_cmd, (void __user*)arg, sizeof(temp_cmd));
            if (rc < 0) {
                return rc;
            }
            if (temp_cmd.padding) {
                dev_err(&mdev->pci_device->dev, "Padding in IOCTL not zero");
                return -EINVAL;
            }
            rc = apply_temp_cmd(mdev, &temp_cmd,  MINION_IOCTL_TEMP_CMD_WRITE == cmd);
            if (rc < 0) {
                return rc;
            }
            return copy_to_user((void __user*)arg, &temp_cmd, sizeof(temp_cmd));
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
        sysfs_remove_group(&mdev->pci_device->dev.kobj, &mdev->tc_attr.thermal_group );
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

static void setup_channel_remapping_memory(struct minion_device_s* mdev)
{
    /* This code is based on an algorithm found in the USB MinION firmware in
     * asicDataRecieverV1.v line 483 */
    u16 physical, logical, masked_logical;
    for (logical = 0; logical < 512; ++logical) {
        masked_logical = logical & ~0x101;
        switch (logical & 0x101) {
        case 0x000: // Line 1 even channels: 0,2,4,6.. 254
            physical = masked_logical | 0x000;
            break;
        case 0x001: // Line 2 even channels: 256, 258, 260 ... 510
            physical = masked_logical | 0x100;
            break;
        case 0x100: // Line 1 odd channels: 1,3,5,7 ... 255
            physical = masked_logical | 0x001;
            break;
        case 0x101: // Line 2 odd channels: 257, 259, 261 ... 511
            physical = masked_logical | 0x101;
            break;
        }

        // write the physical source for each logical channel into the remapper memory
        writew(physical, (u16*)(mdev->ctrl_bar + ASIC_HS_RECEIVER_REMAP) + logical);
    }
}

static ssize_t data_log_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i;
    struct attribute_wrapper* wrapper = container_of(attr, struct attribute_wrapper, attribute);
    struct message_struct* message = wrapper->p_value;
    ssize_t len = 0;
    len += sprintf(buf+len, "tec_value\tfc_temp\thsink_temp\terr_prop\n");
    for (i=0; i < LOG_LEN; ++i) {
        // Make sure we don't overflow the buffer...
        while (len < PAGE_SIZE - 128) {
            len += sprintf(buf+len, "%u\t%u\t%u\t%d\n",
                        readw(&message->data_log[i].tec_value),
                        readw(&message->data_log[i].fc_temp),
                        readw(&message->data_log[i].hsink_temp),
                        readw(&message->data_log[i].err_prop));
        }
    }
    return len;
}

static ssize_t latest_data_log_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    struct attribute_wrapper* wrapper = container_of(attr, struct attribute_wrapper, attribute);
    struct message_struct* message = wrapper->p_value;
    ssize_t len = 0;
    int i = readw(&message->data_log_pointer);  // Index of latest log entry
    if (i < LOG_LEN)
    {
        len = sprintf(buf, "tec_value:  %u\n"
                           "fc_temp:    %u\n"
                           "hsink_temp: %u\n"
                           "err_prop:   %d\n",
                           readw(&message->data_log[i].tec_value),
                           readw(&message->data_log[i].fc_temp),
                           readw(&message->data_log[i].hsink_temp),
                           readw(&message->data_log[i].err_prop));
    } else {
        len = sprintf(buf, "Invalid data_log_pointer: %d", i);
    }
    return len;
}


static ssize_t pid_settings_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i;
    struct attribute_wrapper* wrapper = container_of(attr, struct attribute_wrapper, attribute);
    struct message_struct* message = wrapper->p_value;
    ssize_t len = 0;
    len += sprintf(buf+len, "kp_gain ki_gain kd_gain ni_len sample_t fc_therm_weight asic_therm_weight\n");
    for( i=0; i < NUM_PROFILES; ++i) {
        len += sprintf(buf+len, "%u %u %u %u %u %u %u\n",
                       readw(&message->pid_profile[i].kp_gain),
                       readw(&message->pid_profile[i].ki_gain),
                       readw(&message->pid_profile[i].kd_gain),
                       readw(&message->pid_profile[i].ni_len),
                       readw(&message->pid_profile[i].sample_t),
                       readw(&message->pid_profile[i].fc_therm_weight),
                       readw(&message->pid_profile[i].ch514_weight));
    }

    return len;
}

static ssize_t pid_settings_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    int i;
    struct attribute_wrapper* wrapper = container_of(attr, struct attribute_wrapper, attribute);
    struct message_struct* message = wrapper->p_value;
    int start = 0;
    for (i=0; i < NUM_PROFILES; ++i) {
        unsigned int kp_gain, ki_gain, kd_gain, ni_len, sample_t, fc_therm_weight, ch514_weight;
        unsigned int entries;
        int len;
        entries = sscanf(buf+start,"%u %u %u %u %u %u %d%n",
                         &kp_gain,
                         &ki_gain,
                         &kd_gain,
                         &ni_len,
                         &sample_t,
                         &fc_therm_weight,
                         &ch514_weight,
                         &len);
        if (entries != 7) {
            return -EINVAL;
        }
        start += len;
        writew(kp_gain, &message->pid_profile[i].kp_gain);
        writew(ki_gain, &message->pid_profile[i].ki_gain);
        writew(kd_gain, &message->pid_profile[i].kd_gain);
        writew(ni_len, &message->pid_profile[i].ni_len);
        writew(sample_t, &message->pid_profile[i].sample_t);
        writew(fc_therm_weight, &message->pid_profile[i].fc_therm_weight);
        writew(ch514_weight, &message->pid_profile[i].ch514_weight);
    }
    return (ssize_t)count;
}

static ssize_t show_value(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    struct attribute_wrapper* wrapper = container_of(attr, struct attribute_wrapper, attribute);\
    return sprintf(buf,"%u\n", readw(wrapper->p_value) );\
}

static ssize_t store_value(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    struct attribute_wrapper* wrapper = container_of(attr, struct attribute_wrapper, attribute);
    long value;
    ssize_t status = kstrtol(buf, 10, &value);
    if (status == 0) {
        // value valid
        writew(value, wrapper->p_value);
        status = count;
    }
    return status;
}

/// Declare the attribute and create a function to read it.
#define DECLARE_ATTRIBUTE_READ(ATTR_NAME) \
    static ssize_t ATTR_NAME##_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {\
        return show_value(kobj, attr, buf);\
    }

/// Declare the attribute and create functions to read and write it
#define DECLARE_ATTRIBUTE_READ_WRITE(ATTR_NAME) \
    static ssize_t ATTR_NAME##_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {\
        return show_value(kobj, attr, buf);\
    }\
    static ssize_t ATTR_NAME##_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {\
        return store_value(kobj, attr, buf, count);\
    }



// Declare functions to read and write common attributes
DECLARE_ATTRIBUTE_READ_WRITE(control);
DECLARE_ATTRIBUTE_READ(error);
DECLARE_ATTRIBUTE_READ_WRITE(tec_override);
DECLARE_ATTRIBUTE_READ_WRITE(tec_dead_zone);
DECLARE_ATTRIBUTE_READ(tec_voltage);
DECLARE_ATTRIBUTE_READ(tec_current);
DECLARE_ATTRIBUTE_READ_WRITE(threshold_1);
DECLARE_ATTRIBUTE_READ_WRITE(threshold_2);
DECLARE_ATTRIBUTE_READ_WRITE(threshold_3);

int setup_sysfs_entries(struct minion_device_s* mdev)
{
    // create subdir
    struct kobject* parent = &mdev->pci_device->dev.kobj;

    // associate attribute_wrappers with their data
    struct message_struct* message = (struct message_struct*)(mdev->ctrl_bar + NIOS_MESSAGE_RAM_BASE);
    VPRINTK("message ram base %p",message);

    mdev->tc_attr = (struct thermal_control_sysfs){
        .thermal_group = {
            .name = "thermal_control",
            .attrs = mdev->tc_attr.attributes
        },
        .attributes = {
            &mdev->tc_attr.control.attribute.attr,
            &mdev->tc_attr.error.attribute.attr,
            &mdev->tc_attr.tec_override.attribute.attr,
            &mdev->tc_attr.tec_dead_zone.attribute.attr,
            &mdev->tc_attr.tec_voltage.attribute.attr,
            &mdev->tc_attr.tec_current.attribute.attr,
            &mdev->tc_attr.data_log.attribute.attr,
            &mdev->tc_attr.latest_data_log.attribute.attr,
            &mdev->tc_attr.threshold_1.attribute.attr,
            &mdev->tc_attr.threshold_2.attribute.attr,
            &mdev->tc_attr.threshold_3.attribute.attr,
            &mdev->tc_attr.pid_settings.attribute.attr,
            NULL
        },
        // pointer to data, attribute-defn
        .control = { &message->control_word, __ATTR_RW(control) },
        .error = { &message->error_word, __ATTR_RO(error) },
        .tec_override = { &message->tec_override, __ATTR_RW(tec_override) },
        .tec_dead_zone = { &message->tec_dead_zone, __ATTR_RW(tec_dead_zone) },
        .tec_voltage = { &message->tec_v, __ATTR_RO(tec_voltage)},
        .tec_current = { &message->tec_i, __ATTR_RO(tec_current)},
        .data_log = { message, __ATTR_RO(data_log)},
        .latest_data_log = {message, __ATTR_RO(latest_data_log)},
        .threshold_1 = { &message->profile_thresh[0], __ATTR_RW(threshold_1)},
        .threshold_2 = { &message->profile_thresh[1], __ATTR_RW(threshold_2)},
        .threshold_3 = { &message->profile_thresh[2], __ATTR_RW(threshold_3)},
        .pid_settings = { message, __ATTR_RW(pid_settings)}
    };

    return sysfs_create_group(parent, &mdev->tc_attr.thermal_group);
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

    {
        struct minion_firmware_info_s fw_info;
        get_fw_info(mdev,&fw_info);
        dump_firmware_info(&fw_info);
    }

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

    setup_channel_remapping_memory(mdev);

    setup_sysfs_entries(mdev);
    devm_add_action(&dev->dev, disable_temperature_control, mdev);

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
