#!/bin/sh

# Load the driver
sudo insmod ~/minion.ko

CLASS="ont-minion1c"
DEV_MAJOR=$(awk "\$2==\"$CLASS\" {print \$1}" /proc/devices)

# Make the device node
sudo mknod /dev/flowcell0 c $DEV_MAJOR 0

# Set permissions
sudo chown $(logname) /dev/flowcell0
