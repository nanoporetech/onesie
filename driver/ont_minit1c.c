/**
 * ont_minit1c_reg.h
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
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>

#include "ont_minit1c.h"
#include "ont_minit1c_reg.h"
#include "ont_minit_ioctl.h"

#define ONT_DEBUG
//#define ONT_VERBOSE_DEBUG

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
    #ifdef ONT_VERBOSE_DEBUG
        #define VPRINTK(ARGS...) do {printk(KERN_ERR __FILE__ ":" STRINGIFY(__LINE__)" :" ARGS);} while(0)
    #endif
#endif
#ifndef DPRINTK
    #define DPRINTK(ARGS...) do {} while(0)
#endif
#ifndef VPRINTK
    #define VPRINTK(ARGS...) do {} while(0)
#endif

/*
 * PROTOTYPES
 */

static int __init pci_probe(struct pci_dev *dev, const struct pci_device_id *id);
static void __exit pci_remove(struct pci_dev *dev);
static int minit_file_open(struct inode* inode, struct file *file);
static int minit_file_close(struct inode *inode, struct file *file);
static long minit_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);


/*
 * STRUCTURES
 */
static struct pci_device_id pci_ids[] = {
    { PCI_DEVICE(0x1172, 0xE003) }, // Altera development card
    { 0 }
};

static struct pci_driver minit_driver_ops = {
    .name = ONT_DRIVER_NAME,
    .id_table = pci_ids,
    .probe = pci_probe,
    .remove = pci_remove,
};

struct file_operations minit_fops =
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
        address = minit_dev->ctrl;
        if ((reg_access->offset + reg_access->size) > CTRL_BAR_EXPECTED_SIZE) {
            return -EFAULT;
        }
        break;
    case SPI_BAR:
        address = minit_dev->spi;
        if ((reg_access->offset + reg_access->size) > SPI_BAR_EXPECTED_SIZE) {
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
            writel((u32)reg_access->value, address);
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



/*
 * File OPs
 */

static int minit_file_open(struct inode* inode, struct file *file)
{
    struct minit_device_s* minit_dev;

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
    default:
        printk(KERN_ERR ONT_DRIVER_NAME": Invalid ioctl for this device (%u)\n", cmd);
    }
    return -EIO;
}


static void cleanup_device(void* data) {
    struct pci_dev* dev = (struct pci_dev*)data;
    struct minit_device_s* minit_dev = pci_get_drvdata(dev);
    if (minit_dev) {
        device_table_remove(minit_dev);

        device_destroy(minit_class, MKDEV(minit_major, minit_dev->minor_dev_no));
    }
}


/**
 * This should:
 *  Verify that we're talking to usable minit hardware.
 *  Map bars and create the device data structure.
 *
 */
static int __init pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    int rc;
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

    minit_dev->ctrl  = pcim_iomap_table(dev)[CTRL_BAR];
    minit_dev->spi = pcim_iomap_table(dev)[SPI_BAR];
    DPRINTK("Control bar mapped to %p\n", minit_dev->ctrl);
    DPRINTK("SPI bar mapped to %p\n", minit_dev->spi);

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

    rc = device_table_add(minit_dev->minor_dev_no, minit_dev);

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
