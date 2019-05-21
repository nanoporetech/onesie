# MinIT-1C Linux device-driver

This projoct contains a Linux device-driver for the FPGA firmware that enables interaction with a
MinION flowcell over a PCIe link.

## Getting Started

Build with `make all` and start with `insmod driver/minion.ko`

Check the kernel logs (`dmesg`) to see if the driver has found hardware. Currently the
driver doesn't create a device-node so you have to run through a couple of steps to
do that manually

Look through the list of devices in `/proc/devices` for `ont-minit1c` you need the
number associated with it, that is the major device-number.

Create the device node wherever you feel comfortable

`sudo mknod onsie c <major-no> 0`

If this works there should be a file called "onesie" owned by root. For convenience,
change it to be owned by you.

`sudo chown <username> onesie`

Warning: The major number that the driver uses _can_ change each time driver is
loaded. Though the file will still be there. If the driver stops working check
`/proc/devices` to see that its still using the same major number. If it's changed
delete the device node and create a new one with the correct number

## Driver Internals

The driver is a PCI/PCIe device driver. It is roughly split into three compilation
units:

*  dma.c: handles transferring signal data from the ASIC to memory
*  i2c-altera.c: EEPROM access
*  minon_top.c: driver registration, kernel inteface and everything else

The i2c-altera.c file is a derivative work of drivers/i2c/busses/i2c-altera.c
driver, itself based on the i2c-axxia.c driver. As this code is subject to the
GPLv2 license, the MinIT-1C device driver must be distributed under the GPLv2
license.

Almost all interaction with the driver is through IOCTLs, these are defined in
`minion_ioctl.h` and reasonably well documented. As with all IOCTLs, when being used, padding must zeroed and will be checked by the driver. Macros are provided that give the sizes expected for the structures that communicate data in and out of the IOCTLs.

### Startup/Shutdown

When the driver is started, either through modprobe or insmod, it will allocate
character-device device-numbers and register support for PCI devices with
the vendor/device numbers:

*  0x1ab0/0x0010 Altera development card
*  0x1e59/0x0001 Nanopore Technology official MinION-1C ID

When the PCI system detects cards with this hardware it will run `pci-probe`
this will enable the device, map BARs, set-up DMA masks and interrupts. It will
allocate memory for a driver-internal device-structure `minion_device_s` and
populate that structure with information. After creating a character-device for
the driver, it will set-up the I2C and DMA cores.

Clean-up when the driver is removed is largely handled by manage-resources.
There are some exceptions and these resources are freed from the `cleanup_device`
function that is registered with the managed-resources framework to be run when
the driver exits.

There is a system that links device-nodes numbers (that will be passed when
accessing the driver through its file interfaces) to the `minion_device_s`
structure. These are created, removed and accessed through the `device_table_add`,
`device_table_remove` and `device_table_lookup` functions respectively.

*  TODO: how it should be started and how it should create device nodes

### MinION EEPROM Access

The EEPROM on the MinION flow-cell is accessed over a I2C bus. There is only one
device (the EEPROM) and one master on the bus, so under normal circumstances,
there should be no difficulties with bus arbitration.

The data and clock wires are shared with one of the high-speed links to the ASIC. To access the EEPROM, the ASIC must be placed in reset and the high-speed bus disabled and switched to I2C mode. Analogue power does not need to be enabled to access the EEPROM. Obviously, it is not possible to access the EEPROM whilst the ASIC is operational. So in an attempt to avoid disrupting ASIC operation through EEPROM access, the `switch_link_mode` and `free_link` functions were added. The intention was that these would act like a semaphore that also switches the hardware to/from I2C mode. Unfortunately determining when ASIC access was complete was more complex than originally thought and these functions are not used as effectively as envisaged.

The I2C master is an Altera soft-core, documented in their Embedded Peripherals User Guide UG-01085. The Linux kernel contains a device-driver for the core, unfortunately this driver requires that the core be on a peripheral-bus. The code has been duplicated and modified to support operation though a PCIe bus and also adds support for mixed-mode commands needed for EEPROM access. A number of bugs in the original driver have been fixed. The Linux kernel I2C EEPROM framework could not be used. This framework has a probe phase when started, where it searches for hardware. If a flow-cell is not present when the driver is loaded, then this will not work. The code to implement EEPROM access is not too complicated so has been written from scratch.

Both reads and writes are broken on the 8-byte page boundaries of the EEPROM used in the flow-cells. Reads are implemented with a mixed-mode transfer, the first message writes the address, the second message instructs the core to read the data. Reads should be paced by the EEPROM providing the data and it should be possible to read the entire EEPROM in less than 5 milliseconds.

Writes are a transfer comprised of a single message, the address and the data to write. If the EEPROM is still committing the previous write to memory, it will refuse to acknowledge new commands. In this situation, the driver will wait 5 milliseconds then retry the unacknowledged command. 5 ms is usually sufficient for the write to complete though in many cases the write will complete with needing to retry. After all pages have been written, the driver will send an empty command (a 0-bytes write), The EEPROM acknowledging this indicates that all data has been committed to storage, following reads should not be held-off needing a retry.

### MinION ASIC Control

IOCTLs are provided for

*  accessing the registers in the high-speed receiver core
*  reading and/or writing to the ASIC via its SPI link
*  generic access to registers in the FPGA firmware

The high-speed receiver core contains a number of 16-bit registers. The IOCTL optionally writes to all of these, but will always read all of the registers. Reads are performed after any writes.

The shift-register controls simplifies accessing the ASIC over the SPI links. `switch_link_mode` is used to lock-out the EEPROM and prevent the ASIC from being reset or brought-out-of-reset half way through a transaction. The speed of the SPI link is set by a clock-divider dividing-down the PCIe core clock, the IOCTL communicates a desired clock-speed and the driver will attempt to select a divider to achieve it.

An IOCTL is provided for register-access, this was originally intended for debugging purposes, but is used by MinKNOW to access the ASIC-Control register.

### Data Movement (DMA)

Sequencing current-measurement data is produced by ADCs in the ASIC and communicated over two differential-pair links to the FPGA. The data is re-ordered by the high-speed receiver core, so the order of the channels in the received data matches the order of the channels in the control-data sent over the SPI link. The high-speed receiver core also provides some buffering. Altera DMA firmware comprised of a modular scatter-gather DMA core and a modular scatter-gather DMA pre-fetcher core writes the data into host memory using chained DMA descriptors to describe the transfers.

The API for the transfer, as seen from the perspective of a user-space client is:

1.  Submit one or multiple transfer(s) with buffer location, size, transfer-id, signal-number and process-id (to receive the signal)
1.  (Optionally) Wait for a signal
1.  Enquire as to what's completed

The use of signals is optional, they can interrupt some system-calls, though none of the driver IOCTLs are interruptible. Buffers *must* be aligned on a cache-line boundary at both start and end, they will be rejected by the driver if not.

Though there is a driver for the modular scatter-gather DMA core in the kernel, this has not been used as it doesn't support the pre-fetcher core. The DMA code in this driver is an original work.

The ioctl calls 'queue_data_transfer' with the parameters for the transfer copied from user-space.
ioctl. This:
1.  checks buffer alignment
1.  allocates and populates a "Job" structure used for managing the transfer in the driver
1.  allocates a list of pages associated with the buffer and attaches it to the job structure
1.  locks the pages in memory, so they can't be swapped out to disk during the DMA transfer
1.  creates a scatter-list for the pages, with adjacent pages merged.
1.  maps the scatter-list so the virtual-addresses in transfers are converted to physical-addresses, caches are invalidated and IOMMUs programmed appropriately

This then calls 'create_descriptor_list' this converts the scatter-list into a chain of DMA descriptors stored in coherent memory and allocated from a dma-descriptor-pool. This will split scatter-gather entries when they are too large for the firmware (greater than 2048 bytes.)

DMA descriptors used by the Altera pre-fetcher core contain an "ownership" bit that assigns ownership of each descriptor in the chain to either hardware or software. When the pre-fetcher core encounters a descriptor owned by software it pauses, polling at intervals until the descriptor is owned by hardware. setting a descriptor to be owned by software terminates a DMA descriptor chain.


ready for hardware queue

on hardware and isr(s)

post hardware queue

signals

done queue

whats done ioctl

cancelling

## Tools

## Packaging and Distribution
