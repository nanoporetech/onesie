# QEMU of Minit-1C hardware

This directory contains qemu as a subproject.

The files in the "modifications" directory will be copied into the qemu
directory-tree before a build, overwriting files in the qemu directiory.

## Creating a virtual machine

I haven't found a way to start the modified qemu with networking. So first use
the standard system qemu to build a Ubuntu Xenial x86_64 virtual machine, install packages for
building kernel modules.

To access the Nanopore git repo you need to modify some files:

*  Change _/etc/hosts_, where the machine name is mapped to 127.0.1.1 add
`<machine-name>.oxfordnanolabs.local`

*  Change _/etc/nsswitch.conf_, remove `mdns4_minimal [NOTFOUND=return]` from the...  
`hosts:  files mdns4_minimal [NOTFOUND=return] dns`   
...line to give...  
`hosts:  files dns`

Install the nanopore certificaties with  
```
wget http://deb-repo.oxfordnanolabs.local/apt/pool/non-free/o/ont-ca-certs/ont-ca-certs_2018.04.30-1392589~xenial_all.deb  
sudo dpkg -i ont-ca-certs_2018.04.30-1392589~xenial_all.deb
```
Clone and build this project in the VM, check that the
driver loads, etc.

## Starting virtual machine

To start the virtual machine from the commnd line in the onsie/test/emulation directory.
```
sudo LC_ALL=C PATH=/usr/local/sbin:/usr/local/bin:/usr/bin:/usr/sbin:/sbin:/bin QEMU_AUDIO_DRV=none ;
$(pwd)/qemu/x86_64-softmmu/qemu-system-x86_64 \
-name xenial \
-S \
-machine pc,accel=kvm,usb=off \
-m 4096 \
-realtime mlock=off \
-smp 2,sockets=2,cores=1,threads=1 \
-uuid 51cf2f2a-1610-39cd-5c89-04eca9601b5f \
-no-user-config \
-nodefaults \
-chardev socket,id=charmonitor,path=/var/lib/libvirt/qemu/xenial.monitor,server,nowait \
-mon chardev=charmonitor,id=monitor,mode=control \
-rtc base=utc \
-no-shutdown \
-boot strict=on \
-device piix3-usb-uhci,id=usb,bus=pci.0,addr=0x1.0x2 \
-drive file=/var/lib/libvirt/images/xenial.img,if=none,id=drive-virtio-disk0,format=qcow2,cache=writeback \
-device virtio-blk-pci,scsi=off,bus=pci.0,addr=0x4,drive=drive-virtio-disk0,id=virtio-disk0,bootindex=1 \
-drive if=none,id=drive-ide0-1-0,readonly=on,format=raw \
-device ide-cd,bus=ide.1,unit=0,drive=drive-ide0-1-0,id=ide0-1-0 \
-chardev pty,id=charserial0 \
-device isa-serial,chardev=charserial0,id=serial0 \
-vnc 127.0.0.1:0 \
-device cirrus-vga,id=video0,bus=pci.0,addr=0x2 \
-device virtio-balloon-pci,id=balloon0,bus=pci.0,addr=0x5 \
-device minit-1c,bus=pci.0\
-display gtk
```
This should open a window for the VM. The VM starts paused so un-pause from the menu.

## Modifying the command-line

The command-line for starting the VM can be found in the logs in _/var/log/libvirt/qemu_
add the `-device minit-1c,bus=pci.0` and `-display gtk`, cut out the bits it complains about
## Bits of command-line

```
-device virtio-net-pci,netdev=hostnet0,id=net0,mac=52:54:00:ce:30:78,bus=pci.0,addr=0x3 \

-netdev tap,fd=24,id=hostnet0,vhost=on,vhostfd=25 \
```