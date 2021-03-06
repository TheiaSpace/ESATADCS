Copyright (C) 2017 Theia Space, Universidad Politécnica de Madrid

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


ADCS telemetry packets go here.


# ESAT_ADCSTelemetryPacket

Telemetry packets must implement this interface, which exposes two methods:

  byte packetIdentifier();

  void readUserData(ESAT_CCSDSPacket& packet);

ESAT_ADCS keeps a stack of available telemetry packets.  On each ADCS
update cycle, stack the packets that will generated by the next calls
to ESAT_ADCS.readTelemetry() with ESAT_ADCS.addTelemetryPacket().
ESAT_ADCS.readTelemetry() will remove ADCS telemetry packet objects
from the stack and call their readUserData() methods to fill the
user data fields of the packets.


# Telemetry packet identifiers

  HOUSEKEEPING: 0x00
