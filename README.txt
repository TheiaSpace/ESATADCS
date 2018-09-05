Copyright (C) 2017-2018 Theia Space, Universidad Politécnica de Madrid

This file is part of Theia Space's ESAT ADCS library.

Theia Space's ESAT ADCS library is free software: you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation, either
version 3 of the License, or (at your option) any later version.

Theia Space's ESAT ADCS library is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Theia Space's ESAT ADCS library.  If not, see
<http://www.gnu.org/licenses/>.


ESAT ADCS library version 3.2.0.

Attitude Determination and Control Subsystem (ADCS) for ESAT.

This software runs either as the main program in the ESAT ADCS board
(for the newer model with a standalone microprocessor) or as a library
in the ESAT OBC board (for use together with the legacy ESAT ADCS
board controlled by the ESAT OBC board).  Use it with Arduino 1.8.0 or
newer together with:

  * ESAT ADCS board: the Arduino core for STM32L4-based ESAT
    boards (Theia Space's ESAT Boards (STM32L4)) 1.0.0 or a newer 1.x.y
    version and the ESAT utility library (ESATUtil) version 2.1.0 or a
    newer 2.x.y version.  Use ESAT-ADCS as the target board.  In
    addition, program the On-Board Computer with the option ADCS Code
    Running in ADCS (option ADCS=ADCS).

  * ESAT OBC board: the Arduino core for MSP430-based ESAT boards
    (Theia Space's ESAT Boards (MSP430)) 2.2.0 or a newer 2.x.y
    version, the ESAT utility library (ESATUtil) version 2.1.0 or a
    newer 2.x.y version, and the ESAT OBC library (ESATOBC) version
    4.1.0 or a newer 4.x.y version.  Use the example OBC program,
    ESAT-OBC as the target board and the option ADCS Code Running in
    OBC (option ADCS=OBC).

See the example ADCS program (examples/ADCS/ADCS.ino).  This program
uses the modules of the ADCS library.  This program runs in the ADCS
board.

The src/ directory contains the ADCS library, which has a main module
(ESAT_ADCS) as well as helper modules distributed in subdirectories.
ESAT_ADCS contains the public interface to the ADCS.  Currently, the
ADCS runs as a library used by the On-Board Computer program:

* It calls ESAT_ADCS.begin() to start the ADCS.
* It calls ESAT_ADCS.getApplicationProcessIdentifier() and
  ESAT_ADCS.handleTelecommand() to respond to telecommands sent to the
  ADCS.
* It calls update() to iterate the ADCS loop.
* It calls telemetryAvailable() and readTelemetry() to retrieve ADCS
  telemetry.

The ADCS has different run modes.  A run mode is an object that
implements the ESAT_ADCSRunMode interface.  To change the run mode,
use ESAT_ADCS.setRunMode().

It is possible to define new ADCS telecommands with new telecommand
handlers.  A telecommand handler is an object that implements the
ESAT_ADCSTelecommandHandler interface.  To add new telecommand
handler, use ESAT_ADCS.registerTelecommandHandler().

It is possible to stack new ADCS telemetry packets for emission on
each ADCS cycle.  A telemetry packet object implements the
ESAT_ADCSTelemetryPacket interface.  To stack a new telemetry packet
on a given ADCS cycle, use ESAT_ADCS.addTelemetryPacket().

There are 5 subdirectories in src/ with the libraries that provide the
detailed ADCS functionality:

  * ESAT_ADCS-actuators: low-level actuator control libraries.

  * ESAT_ADCS-controllers: high-level attitude and actuator control
    libraries.

  * ESAT_ADCS-measurements: libraries for the attitude state vector
    and the sensors that measure the attitude state.

  * ESAT_ADCS-run-modes: libraries that handle the different modes of
    operation of the ADCS.

  * ESAT_ADCS-telecommand-handlers: libraries that handle groups of
    telecommands.

  * ESAT_ADCS-telemetry-packets: library that fill telemetry packets.
