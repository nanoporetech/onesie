# MinIT-1C Linux device-driver

## Description

This projoct contains a Linux device-driver for the FPGA firmware that enables interaction with a
MinION flowcell over a PCIe link.

## Starting

Build with `make all` and start with `insmod driver/minit-1c.ko`

Check the kernel logs (`dmesg`) to see if the driver has found hardware. Currently the
driver doesn't create a device-node so you have to run through a couple of steps to
do that manually

Look through the list of devices in `/proc/devices` for `ont-minit1c` you need the
number associated with it, that is the major device-number.

Create the divce node wherever you feel comfortable

`sudo mknod onsie c <major-no> 0`

If this works there should be a file called "onesie" owned by root. For convenience,
change it to be owned by you.

`sudo chown <username> onsie`

Warning: The major number that the driver uses _can_ change each time driver is
loaded. Though the file will still be there. If the driver stops working check
`/proc/devices` to see that its still using the same major number. If it's changed
delete the device node and create a new one with the correct number
