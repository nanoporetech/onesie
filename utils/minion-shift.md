% MINION-SHIFT(1) Version @VERSION@ | Driver Development Utility

NAME
====

**minion-shift** - Read/Write MinION ASIC registers.

SYNOPSIS
========

| **minion-shift** \[_OPTIONS_\] _DEVICE_

DESCRIPTION
===========

**minion-shift** performs a read and optionally a write of the 2259-bit ASIC
register block via the SPI link. Before using this command it is recommended to
enable analogue-power to the ASIC. Communiction with the firmware controlling
the SPI link is by IOCTLs to _DEVICE_.

Command-data read from the ASIC is sent to standard-output as either a byte-
stream, or if the **\-\-hex** option is enabled, comma separated hexadecimal
data. The comma-separated hexadecimal data output is the correct format for
using as input with the **-write** option. The clock-frequency defaults to
1-MHz any frequency specified is subject to the resolution of the clock-divider
and should be in the range of 62.5-MHz to 1-MHz. Where possible, unachievable
clock-frequencies will be adjusted rather than rejected.

In almost all scenarios, the **\-\-enable** and **\-\-start** options should be
selected.

OPTIONS
=======

-x, \-\-hex

:   Output will be in hexadecimal csv. If this flag is not set, then output
will be in binary. This can be saved to a file or piped to **shift-decode**
for translation into a human-readable format.

-e, \-\-enable

:   Set the enable bit.

-s, \-\-start

:   Set the start-bit.

-f, \-\-frequency _frequency_

:   Set the interface clock-frequency in Hz (default approx 1 MHz.)

-w, \-\-write

:   Write comma separated bytes to the ASIC read from standard-input.


SEE ALSO
========

**shift-decode**(1) Translate the output of this command to human-readable form.

AUTHOR
======

Oxford Nanopore Technologies PLC <info@nanoporetech.com>
