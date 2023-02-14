% MINION-DMA(1) Version @VERSION@ | Driver Development Utility

NAME
====

**minion-dma** - Exercise MinION mk1C device-driver data acquisition

SYNOPSIS
========

| **minion-dma** \[_OPTIONS_\] DEVICE

DESCRIPTION
===========

**minion-dma** performs DMA transfers of acquisition data from the flow-cell and
sends any data received to standard out as a bytes-stream.

The \-\-size option controls the size of the buffers used for transfers. The driver
will reject any buffers that do not both start and end on a cache-line boundary
(128-bytes on aarch64.) To enable buffers to start and end in the right place,
their length must be a multiple of the cache-line size. In addition to the
cache-line size requirement, it is recommended that the buffers be a multiple
of the frame-size (1056-bytes.) The smallest buffer that meets both these
requirements is 4224-bytes, four frames. The default size doesn't meet any of
these requirements.

Prior to running **minion-dma**, it is recommended to:

1.  Enable analogue-power and set a bus frequency
2.  Assert and de-assert reset on the high-speed receiver core.

The number of buffers transferred is specified by the **\-\-no-transfers** option.
After all buffers have been transferred, **minion-dma** will exit. If you wish to
continuously transfer data use the **\-\-stream** option.

Data is output as a bytes-stream, not normally intelligible with out a some
other tool such as **hexdump**


OPTIONS
=======

-s, \-\-size _size_

:   _Size_ of each transfer, defaults to 514*2-bytes.

-n, \-\-no-transfers _transfers_

:   Number of _transfers_, (default 1)

\-\-stream

:   Transfer data repeatedly until further notice. If this is set \-\-no-transfers will be ignored.

-q, \-\-max-queue _maximum_

:   _Maximum_ number of transfers to queue at once (default 8)

-p, \-\-poll

:   Use polling rather than signals to detect when transfers have completed.

-r, \-\-reset

:   Reset the High-Speed receiver when starting.

EXAMPLES
========

$ minion-dma /dev/flowcell0 -s 4224 -r | hexdump -C

:   Read four frames and use hexdump to make readable.

$ minion-dma /dev/flowcell0 -s 135168 -r \-\-stream > captured-data

:   Stream 128-frame buffers to a file.

SEE ALSO
========

**dma-audit**(1), **hexdump**(1), **xxd**(1)

AUTHOR
======

Oxford Nanopore Technologies PLC <info@nanoporetech.com>
