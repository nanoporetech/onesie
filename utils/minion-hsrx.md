% MINION-HSRX(1) Version @VERSION@ | Driver Development Utility

NAME
====

**minion-hsrx** - Read/Write registers in MinION-mk1C 

SYNOPSIS
========

| **minion-hsrx** \[_OPTIONS_\] _DEVICE_

DESCRIPTION
===========

**minion-hsrx** sends ioctls to read and optionally write parts-of the
registers in the high-speed receiver core. The values read from the registers
are written to standard output as either a byte-stream (needing a hex-viewer
to read) or optionally as hexadecimal.

OPTIONS
=======

-e, \-\-enable

:   Set the enable-bit in hs-rx word-0.

-r, \-\-sync-reset

:   Set the reset-bit in hs-rx word-0.

-x, \-\-hex

:   Output registers read as hexadecimal rather than a byte-stream.

-f, \-\-frames <no-frames>

:   Set the number of frames of data between "End-of-Packet" signals.

-h, \-\-human

:   Output the registers in (vaguely) human-readable format.

EXAMPLES
========

$ minon-hsrx -h /dev/flowcell0

:   Show the current state of the HSRX core registers.


SEE ALSO
========

**hexdump**(1), **xxd**(1)

AUTHOR
======

Oxford Nanopore Technologies PLC <info@nanoporetech.com>
