Copyright (C) 2017, 2018 Theia Space, Universidad Politécnica de Madrid

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


ESAT_ADCS contains the public interface to the ADCS.  This library
exposes the following high-level ADCS functionality:

  * ESAT_ADCS.begin(): start the ADCS.
  * ESAT_ADCS.getApplicationProcessIdentifier(): return the application
    process identifier of the ADCS application process.
  * ESAT_ADCS.handleTelecommand(): respond to telecommands sent to the
    ADCS.
  * ESAT_ADCS.update(): iterate the ADCS loop.
  * ESAT_ADCS.telemetryAvailable(): return true if there are new
    telemetry packets available.
  * ESAT_ADCS.readTelemetry(): retrieve ADCS telemetry.

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

Run modes and telemetry packets may query ESAT_ADCS for information:

  * ESAT_ADCS.attitudeStateVector(): return the current attitude state
    vector.
  * ESAT_ADCS.period(): return the period from the period call to
    ESAT_ADCS.update() to the current call to ESAT_ADCS.update().
  * ESAT_ADCS.runModeIdentifier(): return the identifier number of the
    current run mode.

There are additional functions that the main ADCS program may use:

  * ESAT_ADCS.disableUSBTelecommands(): disable the reception of
    telecommands through the USB interface.
  * ESAT_ADCS.disableUSBTelemetry(): disable the emission of telemetry
    through the USB interface.
  * ESAT_ADCS.enableUSBTelecommands(): enable the reception of
    telecommands though the USB interface.
  * ESAT_ADCS.enableUSBTelemetry(): enable the emission of telemetry
    through the USB interface.
  * ESAT_ADCS.readTelecommand(): read an incoming telecommand.
  * ESAT_ADCS.respondToI2CRequests(): respond to telemetry and
    telecommand requests coming from the I2C bus.
  * ESAT_ADCS.writeTelemetry(): send a telemetry packet through the
    USB interface.
