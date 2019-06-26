#!/bin/sh

# A script to simplify writing to the control file, which needs sudo to write to
# Run this as:
#     $ sudo ./temperature_control.sh 1
echo $1 > /sys/class/ont-minion1c/ont-minion1c_0/device/thermal_control/control
