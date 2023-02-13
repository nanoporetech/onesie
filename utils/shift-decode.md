% SHIFT-DECODE(1) Version @VERSION@ | Driver Development Utility

NAME
====

**shift-decode** - Convert MinION ASIC register data into human-readable form.

SYNOPSIS
========

| **shift-decode** \[_OPTIONS_\]

DESCRIPTION
===========

**shift-decode** accepts the byte-stream from minion-shift and converts all or
part of it into human-readable form.

OPTIONS
=======

-x, \-\-hex

:   Accept input in comma-separated hexadecimal format.

-o, \-\-otp

:   Only decode the OTP part of the shift register.

-n, \-\-not-input-select

:   Do not decode the input selection for the channels.


EXAMPLES
========

$ minion-shift -s -e /dev/flowcell0 | shift-decode

Decode the output of **minion-shift** reading the ASIC registers.

SEE ALSO
========

**minion-shift**(1) Utility read the MinION ASIC registers.

AUTHOR
======

Oxford Nanopore Technologies PLC <info@nanoporetech.com>
