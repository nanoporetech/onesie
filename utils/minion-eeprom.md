% MINION-EEPROM(1) Version @VERSION@ | Driver Development Utility

NAME
====

**minion-eeprom** - Read/Write EEPROM on the MinION flow-cell

SYNOPSIS
========

| **minion-eeprom** \[_OPTIONS_\] _DEVICE_

DESCRIPTION
===========

minion-eeprom is used to read or write part or the whole of the EEPROM on the
MinION flow-cell. The contents of the flow-cell will be written to standard
output when reading and read from standard input when writing.

The EEPROM contents will be written to standard output as a byte-stream so it is
recommended to pipe the data into a hex-viewer such as hexdump or xxd.


OPTIONS
=======

-r, \-\-read

:   Read from EEPROM to standard output.

-w, \-\-write

:   Write data from standard input to EEPROM.

-s, \-\-start _start-address_

:   _Start-address_ in decimal or hex if starting with 0x. Defaults to 0.

-l, \-\-length <length>

:   _Length_ of transfer in decimal or hex if starting with 0x. Defaults to the
maximum possible length given the size of the EEPROM and the _start-address_.

EXAMPLES
========

$ minit-eeprom -r -s 0xfc -l 4 /dev/flowcell0 | hexdump -C

:   Read the EEPROM serial number:

$ minit-eeprom -w /dev/flowcell0 < my-eeprom-contents

:   Write the entire EEPROM with the contents of a file

SEE ALSO
========

**hexdump**(1), **xxd**(1)

AUTHOR
======

Oxford Nanopore Technologies PLC <info@nanoporetech.com>
