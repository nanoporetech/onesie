% DMA-AUDIT(1) Version @VERSION@ | Driver Development Utility

NAME
====

**dma-audit** - basic auditing on a byte-stream from **minion-dma**.

SYNOPSIS
========

| **dma-audit** \[**OPTIONS**\]

DESCRIPTION
===========

**dma-audit** accepts the byte-stream from **minion-dma** through standard input
and does some basic auditing on the meta-data to verify that the DMA is
operating correctly.

**dma-audit** checks for:

*  correctly incrementing frame-numbers
*  0xCAFEBABE end-of-frame marker
*  changes in sampling-frequency, ASIC-id, bias-voltage, heat-sink temperature
   and ASIC configuration-id (aka command-id.)

OPTIONS
=======

-x, --hex

:   Accept input in comma separated hexadecimal format.

-o, --otp

:   Only decode the OTP part of the shift register.

-n, --not-input-select

:   Do not decode the input select for the channels.


EXAMPLES
========

$ dma-audit < captured-data

:   Audit a file containing previously captured data.

$ minion-dma /dev/flowcell0 -s 84480 -r --stream | dma-audit

:   Audit data as it is being acquired from the hardware.

SEE ALSO
========

**minion-dma**(1) Utility to acquire data.

AUTHOR
======

Oxford Nanopore Technologies PLC <info@nanoporetech.com>
