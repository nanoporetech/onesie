% MINION-TEMP(1) Version @VERSION@ | MinION mk1C Temperature Control

NAME
====

**minion-temp** - Reads Minion mk1C temperature sensors and control the set-point.

SYNOPSIS
========

| **minion-temp**  _DEVICE_ \[_OPTIONS_\]

DESCRIPTION
===========

Reads the temperature sensors and controls the set-point of the control-loop
regulating flow-cell temperature. **minion-temp** communicates with the 
firmware controlling the temperature hardware through IOCTLs to _DEVICE_.

A Peltier device effects heat transfer between the thermal-pad and a heat-sink
that transfers heat to/from airflow through the device. When run, the program
will perform the optional control functions (setting or disabling thermal
control) then return the current desired temperature and the temperatures of
the heat-sink and the thermal-pad (used as a proxy for the flow-cell.) It will
also communicate any errors reported by the thermal control software.

OPTIONS
=======

-s, \-\-set _temperature_

: Set the desired _temperature_ (set-point) of the flow-cell in Celsius and
enable temperature control.

-f, \-\-off

:   Disable temperature control.

-r, \-\-restart

:   Restart temperature controller.

EXAMPLES
========

$ minion-temp /dev/flowcell0 -s 35.2

:   Enable temperature control and set the desired temperature to 35.2C.

$ minion-temp /dev/flowcell0 -r

:   Restart the temperature controller; temperature control will be disabled.

$ minion-temp /dev/flowcell0

:   Report temperatures and errors.


AUTHOR
======

Oxford Nanopore Technologies PLC <info@nanoporetech.com>
