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

There are 5 subdirectories with the libraries that provide the
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
