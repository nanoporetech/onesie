% MINION-REG(1) Version @VERSION@ | Driver Development Utility

NAME
====

**minion-reg** - Read and write registers in the MinION mk1C firmware.

SYNOPSIS
========

| **minion-reg** _DEVICE_ _BAR_
| **minion-reg** _DEVICE_ _BAR_ _OFFSET_ \[_VALUE_\]
| **minion-reg** _DEVICE_ _BAR_ _OFFSET_ \[read8|read16|read32|read64\]
| **minion-reg** _DEVICE_ _BAR_ _OFFSET_ \[write8|write16|write32|write64\] _VALUE_

DESCRIPTION
===========

Read and write registers in the MinION-mk1C firmware. Register accesses are
done through IOCTLs to _DEVICE_. There are 2 _BAR_s (PCI Basic Address
Register groups), 0 and 2. If no _OFFSET_ is specified, then **minion-reg** will
read all the registers in the BAR. _OFFSET_ is in bytes and can be specified in
decimal, octal or hexadecimal. To write to the register, specify a _VALUE_,
again this can be in decimal, octal or hexadecimal.

OPTIONS
=======

read8, read16, read32, read64

:   Read 8, 16, 32 or 64 bit registers. Defaults to 32-bit reads.

write8, write16, write32, write64

:   Write 8, 16, 32 or 64 bit registers. Defaults to 32-bit writes.

EXAMPLES
========

$ minion-reg /dev/flowcell0 0

:   Dump all registers in BAR 0

AUTHOR
======

Oxford Nanopore Technologies PLC <info@nanoporetech.com>
