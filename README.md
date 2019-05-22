# MinION-mk1C Linux device-driver

This project contains a Linux device-driver for the FPGA firmware that enables interaction with a
MinION flowcell over a PCIe link.

## Getting Started

Building is more complicated than `make all` due to the need to access the MinIT kernel include files, these are not installed in the MinIT/MinION-1C by defaults and there are no packages available for them. At the moment, the driver has to be cross-compiled and the utilities compiled natively.

### Build Driver

Set-up a bash shell with a compilation environment. Follow the [MinIT BSP Build / Install instructions](https://wiki.oxfordnanolabs.local/pages/viewpage.action?pageId=82990823), but after the "Set up environment for Kbuild" step, stop and modify `linux-minit-build/make_all.sh` and comment out the last two lines that delete the "source" and "build" directories. Continue with the `source $BASE/linux-minit-build/make_all.sh` step. After this has been done once, subsequently, you just need to set the `BASE` environment variable and run `linux-minit-build/build_env.sh`

Using the shell environment set-up above, cd to the `onesie/driver` directory and run

`KERNELRELEASE=4.4.38-minit KERNELDIR=~/code/minit-kernel/linux_build/kernel_module/lib/modules/4.4.38-minit make`

A `minion.ko` file should be built.

### Build Utils

Clone this repository on the minion-1c, cd to `onesie/utils`, run `make`.

### Use on MinION-1C

scp the driver onto the minion-1c and start with `insmod minion.ko`

Check the kernel logs (`dmesg`) to see if the driver has found hardware. Currently the
driver doesn't create a device-node so you have to run through a couple of steps to
do that manually

Look through the list of devices in `/proc/devices` for `ont-minit1c` you need the
number associated with it, that is the major device-number.

You can create the device node wherever you feel comfortable and call it whatever you want, but MinKNOW looks for it as `/dev/flowcell0`. If you do create the device node in `/dev` you will have to recreate it every time you boot the minion-1c as that file-system is not persistent.

`sudo mknod /dev/flowcell0 c <major-no> 0`

If this works there should be a file called `/dev/flowcell0` owned by root. For convenience,
change it to be owned by you.

`sudo chown <username> /dev/flowcell0`

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
2.  (Optionally) Wait for a signal
3.  Enquire as to what's completed

The use of signals is optional as they can interrupt some system-calls (though none of the driver IOCTLs are interruptible.) Buffers *must* be aligned on a cache-line boundary at both start and end, they will be rejected by the driver if not.

Though there is a driver for the modular scatter-gather DMA core in the kernel, this has not been used as it doesn't support the pre-fetcher core. The DMA code in this driver is an original work.

The ioctl calls `queue_data_transfer` with the parameters for the transfer copied from user-space.
ioctl. This:

1.  checks buffer alignment
2.  allocates and populates a "job" structure used for managing the transfer in the driver
3.  allocates a list of pages associated with the buffer and attaches it to the job structure
4.  locks the pages in memory, so they can't be swapped out to disk during the DMA transfer
5.  creates a scatter-list for the pages, with adjacent pages merged.
6.  maps the scatter-list so the virtual-addresses in transfers are converted to physical-addresses, caches are invalidated and IOMMUs programmed appropriately

This then calls `create_descriptor_list` this converts the scatter-list into a chain of DMA descriptors stored in coherent memory and allocated from a DMA-descriptor-pool. This will split scatter-gather entries when they are too large for the firmware (greater than 2048 bytes.)

DMA descriptors used by the Altera pre-fetcher core contain an "ownership" bit that assigns ownership of each descriptor in the chain to either hardware or software. When the pre-fetcher core encounters a descriptor owned by software it pauses, polling at intervals until the descriptor is owned by hardware. Setting a descriptor to be owned by software terminates a DMA descriptor chain. `create_descriptor` will allocate a terminal descriptor to finish the chain.

Next the code moves into `submit_transfer_to_hw_or_queue`. This looks at the state of the DMA cores, if they're idle, it will stop the pre-fetcher core. Free the terminal descriptor that the pre-fetcher is stopped on polling, program the pre-fetcher with the address of the first descriptor in the DMA chain. To aid attaching jobs to the end of the descriptor chain, the address of the terminal-descriptor is recorded in the device structure. Should the DMA core be found to be busy, the driver must "tack" the additional DMA descriptors onto the end of the chain. The chain ends with the terminal-descriptor this cannot be replaced as there is the chance of a race between deciding to replace it and the pre-fetcher core reading it. To join the chains, the first descriptor in the new chain is copied into the terminal descriptor, with the word containing the ownership-bit copied last. This changes the ownership to of that descriptor to hardware and the transfer should be performed. The driver's link to the terminal descriptor is updated and the original first-descriptor for the new job that is no-longer needed is freed.

Jobs are tracked in the driver by moving them between three lists:

*  `transfers_on_hardware`: jobs that are in the descriptor chain either running on or waiting to run on the DMA cores. They will also be in this list for a brief while after the transfer is complete.
*  `post_hardware`: after the transfer has finished, the bottom-half interrupt handler will identify which jobs are complete and move them to this queue
*  `transfers_done`: when all of the operating-system housekeeping is complete, transfers will be placed in this list waiting for user-space to enquire what's done.

These lists are protected with spin-locks the hardware_lock acts as a mutex for accesses to the DMA cores, the `transfers_on_hardware` and the `post_hardware` lists. The `done_lock` guards the `transfers_done` list.

The last descriptor for a job is prepared with the Transfer Complete IRQ Enable bit set. The interrupt is shared with the I2C hardware, so a handler in minion_top.c, `minion_isr_quick` will call handlers in both the I2C and DMA code to establish the source of the interrupt. `dma_isr_quick` will only establish the need to run the bottom-half interrupt handler `dma_isr`. `dma_isr` will iterate through the list of jobs on the hardware. As the hardware finishes each descriptor, it will flip the ownership bit from hardware to software. `dma_isr` checks that all the descriptors for the job in the list it is examining are now "owned" by software and that all the data for the job has been transferred. This will identify the job as "finished" and it will be moved onto the `post_hardware` list. A work-queue task, `post_transfer`.

`post_transfer` largely does the reverse of the tasks performed by `queue_data_transfer`, for each job in the `post_hardware` queue:

1.  free the descriptor list entries
2.  un-map the scatter-list, performing further cache operations and free-up IOMMU resources.
3.  free the scatter-list as it is no-longer needed.
4.  mark pages as "dirty" so any (now incorrect) copies in swap are discarded
5.  release the pages so once again, they can be swapped out to disk if memory is low
6.  free the list of pages that represent the buffer (not the pages themselves, just a list of them).
7.  move the job from the `post_hardware` queue to the `transfers_done` list
8.  if required, send the required signal to the nominated process.

The user-space application (MinKNOW) sends the `MINION_IOCTL_WHATS_COMPLETED` ioctl to enquire which jobs are complete. The transfer id and status of the jobs in the `transfers_done` list are copied into the buffer for the ioctl and the job structure is destroyed.

The `cancel_data_transfer` function will reset the DMA hardware, and dispose of all the jobs in the various queues. It requires that new jobs shall not be submitted when in the reset code. Note that closing the device-node file will have the same effect as sending the ioctl `MINION_IOCTL_CANCEL_TRANSFERS`. This is to prevent the DMA writing into memory that has been freed and reassigned.

## Tools
The utils director contains a number of tools for debugging the driver and firmware:

*  minion-dma
*  minion-eeprom
*  minion-hsrx
*  minion-reg
*  minion-shift
*  dma-audit

### minion-reg

`minion-reg` performs read and write accesses to registers in the firmware via an ioctl. It provides an option to read all the registers in a BAR.

Usage:

1.  `minion-reg <device> <bar>`

    Read all registers in the specified bar

2.  `minion-reg <device> <bar> <offset> [value]`

    Read or when value is provided, write the 32-bit register at specified byte-offset

3.  `minion-reg <device> <bar> <offset> [read8/16/32/64]`

    Read the register at specified byte-offset with the optionally specified length. If the length is not specified then it defaults into usage 2

4.  `minion-reg <device> <bar> <offset> [write8/16/32/64] <value>`

    Write the register at specified byte-offset with the specified length. If the length is not specified then it defaults into usage 2

`<device>` is the device-node

To read multiple registers, but not all the registers in a bar, use a `bash` for-loop, for example to read 10 32-bit registers starting at address 0x04000400 in BAR 2:

`for offset in {0..9}; do minion-reg /dev/flowcell0 2 $(( 0x04000400 + ${offset} * 4 )) ; done`

### minion-eeprom

`minion-eeprom` is used to read or write part or the whole of the EEPROM on the MinION flow-cell. The contents of the flow-cell will be written to standard output when reading and read from standard input when writing.

Usage: `minit-eeprom [-rw] [-s start] [-l length] <device>`

| | |
| --- | --- |
| `-r`, `--read` | read from EEPROM to standard output|
| `-w`, `--write` | write data from standard input to EEPROM |
| `-s`, `--start <offset bytes>` | start address in decimal or hex if starting with 0x. defaults to 0 |
| `-l`, `--length <size bytes>` | length of transfer in decimal or hex if starting with 0x. defaults to max length |

The EEPROM contents will be written to standard output as a byte-stream so it is recommended to pipe the data into a hex-viewer such as `hexdump` or `xxd`.

For example, to read the EEPROM serial number:

`minit-eeprom -r -s 0xfc -l 4 /dev/flowcell0 | hexdump -C`

To write the entire EEPROM from a file:

`minit-eeprom -w /dev/flowcell0 < my-eeprom-contents`

### minion-hsrx

`minion-hsrx` sends ioctls to read and optionally write parts-of the registers in the high-speed receiver core. The values read from the registers are written to standard output as either a byte-stream (needing a hex-viewer to read)  or optionally as hexadecimal.

Usage: `minit-hsrx [-erxf] <device>`

| | |
| --- | --- |
| `-e`, `--enable` | set the enable-bit in hs-rx word-0 |
| `-r`, `--sync-reset` | set the reset-bit in hs-rx word-0 |
| `-x`, `--hex` | output registers read as hexadecimal rather than a byte-stream |
| `-f`, `--frames <no-frames>` | set the number of frames of data between "End-of-Packet" signals |

`<device>` is the device-node

### minion-shift

`minion-shift` performs a read and optionally a write of the 2259-bit ASIC command via the SPI link. Before using this command it is recommended to enable analogue-power to the ASIC.

Usage: `minion-shift [-x] [-e] [-s] [-f frequency ] <device>`

| | |
| --- | --- |
| `-x`, `--hex` | Output will be in hexadecimal csv |
| `-e`, `--enable` | Set the enable bit |
| `-s`, `--start` | Set the start-bit |
| `-f`, `--frequency <frequency>` | Set the interface clock-frequency in Hz (default approx 1 MHz) |
| `-w`, `--write` | Write comma separated bytes from standard-input |

`<device>` is the device-node

Command-data read from the ASIC is sent to standard-output as either a byte-stream, or if the `-x`/`--hex` option is enabled, comma separated hexadecimal data. The comma-separated hexadecimal data output is the correct format for using as input with the `-w` option. The clock-frequency defaults to 1-MHz any frequency specified is subject to the resolution of the clock-divider and should be in the range of 62.5-MHz to 1-MHz. Where possible, unachievable clock-frequencies will be adjusted rather than rejected. 

In almost all scenarios, the `--enable` and `--start` options should be selected.

### minion-dma

`minion-dma` performs DMA transfers of acquisition data from the flow-cell and sends any data received to standard out as a bytes-stream.

`minion-dma [-s size] [-n number of transfers] [-q max queue size ] [-spr] <device>`

| | |
| --- | --- |
| `-s`, `--size <size>` | Size of each transfer, defaults to 514*2-bytes, see below! |
| `-n`, `--no-transfers <no transfers>` | Number of transfers, (default 1) |
| `--stream` | Transfer data repeatedly until further notice. If this is set `--no-transfers` will be ignored. |
| `-q`, `--max-queue <max queue-size>` | Maximum number of transfers to queue at once (default 8) |
| `-p`, `--poll` | Use polling rather than signals to detect when transfers have completed |
| `-r`, `--reset` | Reset the High-Speed receiver when starting |

The `--size` option controls the size of the buffers used for transfers. The driver will reject any buffers that do not both start and end on a cache-line boundary (128-bytes on aarch64.) To enable buffers to start and end in the right place, their length must be a multiple of the cache-line size. In addition to the cache-line size requirement, it is recommended that the buffers be a multiple of the frame-size (1056-bytes.) The smallest buffer that meets both these requirements is 4224-bytes, four frames. The default size doesn't meet any of these requirements.

Prior to running minion-dma, it is recommended to:

1.  Enable analogue-power and set a bus frequency
2.  Assert and de-assert reset on the high-speed receiver core.

The number of buffers transferred is specified by the `--no-transfers` option. After all buffers have been transferred, `minion-dma` will exit. If you wish to continuously transfer data use the `--stream` option.

Data is output as a bytes-stream, not normally intelligible with out a some other tool such as `hexdump`, eg:

`minion-dma /dev/flowcell0 -s 4224 -r | hexdump -C` read four frames and use `hexdump` to make readable.

`minion-dma /dev/flowcell0 -s 135168 -r --stream > captured-data` stream 128-frame buffers to a file.

### dma-audit

`dma-audit` accepts the byte-stream from `minion-dma` through standard input and does some basic auditing on the meta-data to verify that the DMA is operating correctly, eg:

`dma-audit < captured-data` Audit a file containing previously captured data.

`minion-dma /dev/flowcell0 -s 84480 -r --stream | dma-audit` Audit data as it is being acquired from the hardware.

`dma-audit` checks for:

*  correctly incrementing frame-numbers
*  0xcafebabe end-of-frame marker
*  changes in sampling-frequency, ASIC-id, bias-voltage, heat-sink temperature and ASIC configuration-id (aka command-id.)

## Packaging and Distribution


