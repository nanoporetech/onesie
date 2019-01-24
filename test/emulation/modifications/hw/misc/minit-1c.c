/*
 * QEMU PCI MINIT device,
 * This simulation is a derivative work of PCI test device
 *
 * Copyright (c) 2012 Red Hat Inc.
 * Author: Michael S. Tsirkin <mst@redhat.com>
 * Author: Richard Crewe <richard.crewe@nanoporetech.com>
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/event_notifier.h"
#include "sysemu/kvm.h"

typedef struct PCITestDevHdr {
    uint8_t test;
    uint8_t width;
    uint8_t pad0[2];
    uint32_t offset;
    uint8_t data;
    uint8_t pad1[3];
    uint32_t count;
    uint8_t name[];
} PCITestDevHdr;

typedef struct IOTest {
    MemoryRegion *mr;
    EventNotifier notifier;
    bool hasnotifier;
    unsigned size;
    bool match_data;
    PCITestDevHdr *hdr;
    unsigned bufsize;
} IOTest;

#define IOTEST_DATAMATCH 0xFA
#define IOTEST_NOMATCH   0xCE

#define CTRL_BAR_NO 0
#define SPI_BAR_NO 2
#define CTRL_BAR_SIZE (0x01009000 + 1024)
#define SPI_BAR_SIZE 64


typedef struct PCI_MINIT_State {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    MemoryRegion dmaio;
    MemoryRegion nnio;
    int current;
} PCI_MINIT_State;

#define TYPE_PCI_MINIT "minit-1c"

#define PCI_MINIT(obj) \
    OBJECT_CHECK(PCI_MINIT_State, (obj), TYPE_PCI_MINIT)

#define PCI_VENDOR_ID_ONT   0x1172
#define PCI_DEVICE_ID_MINIT  0xe003

typedef struct {
    uint32_t status_desc_base_lo;
    uint32_t status_desc_base_hi;
    uint32_t status_desc_fifo_lo;
    uint32_t status_desc_fifo_hi;
    uint32_t last_ptr;
    uint32_t table_size;
    uint32_t control;
} __attribute__(( packed )) dma_controller_t;

typedef union {
    uint32_t buffer[sizeof(dma_controller_t) / sizeof(uint32_t)];
    dma_controller_t reg;
} dma_controller_regs;

static void minit_pci_reset(PCI_MINIT_State *d)
{
    if (d->current == -1) {
        return;
    }
//    minit_pci_stop(&d->tests[d->current]);
    d->current = -1;
}

static void minit_pci_write(void *opaque, hwaddr addr, uint64_t val,
                  unsigned size, int type)
{
    //PCI_MINIT_State *d = opaque;
}

static uint64_t minit_pci_read(void *opaque, hwaddr addr, unsigned size)
{

    //PCI_MINIT_State *d = opaque;
    return 0;
}

static uint64_t minit_pci_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = 0;
    val = minit_pci_read(opaque, addr,size);
    printf("dma read [%016lx] => %lx (size %d)\n", addr, val, size);
    return val;
}

static uint64_t minit_pci_nn_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val = 0;
    val = minit_pci_read(opaque, addr,size);
    printf("nn read [%016lx] => %lx (size %d)\n", addr, val, size);
    return val;
}

static void minit_pci_dma_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    printf("nn write [%016lx] <= %lx (size %d)\n", addr, val, size);
    minit_pci_write(opaque, addr, val, size, 0);
}

static void minit_pci_nn_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    printf("nn write [%016lx] <= %lx (size %d)\n", addr, val, size);
    minit_pci_write(opaque, addr, val, size, 1);
}

static const MemoryRegionOps minit_pci_dma_ops = {
    .read = minit_pci_dma_read,
    .write = minit_pci_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static const MemoryRegionOps minit_pci_nn_ops = {
    .read = minit_pci_nn_read,
    .write = minit_pci_nn_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static int minit_pci_init(PCIDevice *pci_dev)
{
    PCI_MINIT_State *d = PCI_MINIT(pci_dev);
    uint8_t *pci_conf;

    pci_conf = pci_dev->config;

    pci_conf[PCI_INTERRUPT_PIN] = 0; /* no interrupt pin */

    memory_region_init_io(&d->dmaio, OBJECT(d), &minit_pci_dma_ops, d,
                          "pci-minit-dmaio", CTRL_BAR_SIZE);
    memory_region_init_io(&d->nnio, OBJECT(d), &minit_pci_nn_ops, d,
                          "pci-minit-nnio", SPI_BAR_SIZE);
    pci_register_bar(pci_dev, CTRL_BAR_NO, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->dmaio);
    pci_register_bar(pci_dev, SPI_BAR_NO, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->nnio);

    d->current = -1;

    return 0;
}

static void
minit_pci_uninit(PCIDevice *dev)
{
    PCI_MINIT_State *d = PCI_MINIT(dev);
//    int i;

    minit_pci_reset(d);
/*
    for (i = 0; i < IOTEST_MAX; ++i) {
        if (d->tests[i].hasnotifier) {
            event_notifier_cleanup(&d->tests[i].notifier);
        }
        g_free(d->tests[i].hdr);
    }
    g_free(d->tests);
*/
    memory_region_destroy(&d->dmaio);
    memory_region_destroy(&d->nnio);
}

static void qdev_minit_pci_reset(DeviceState *dev)
{
    PCI_MINIT_State *d = PCI_MINIT(dev);
    minit_pci_reset(d);
}

static void minit_pci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = minit_pci_init;
    k->exit = minit_pci_uninit;
    k->vendor_id = PCI_VENDOR_ID_ONT;
    k->device_id = PCI_DEVICE_ID_MINIT;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_OTHERS;
    dc->desc = "MINIT simulator";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->reset = qdev_minit_pci_reset;
}

static const TypeInfo minit_pci_info = {
    .name          = TYPE_PCI_MINIT,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCI_MINIT_State),
    .class_init    = minit_pci_class_init,
};

static void minit_pci_register_types(void)
{
    type_register_static(&minit_pci_info);
}

type_init(minit_pci_register_types)
