/*
 * Copyright (C) 2017 Theia Space, Universidad Politécnica de Madrid
 *
 * This file is part of Theia Space's ESAT ADCS library.
 *
 * Theia Space's ESAT ADCS library is free software: you can
 * redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Theia Space's ESAT ADCS library is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's ESAT ADCS library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef ESAT_ADCSHousekeepingTelemetry_h
#define ESAT_ADCSHousekeepingTelemetry_h

#include <Arduino.h>
#include <ESAT_CCSDSPacket.h>
#include "ESAT_ADCS-telemetry-packets/ESAT_ADCSTelemetryPacket.h"
#include "ESAT_ADCS-measurements/ESAT_AttitudeStateVector.h"

// ADCS housekeeping telemetry.
// Use the public instance ESAT_ADCSHousekeepingTelemetryPacket.
//
// This packet has the following contents:
// * Mode of operation.
// * Attitude state vector.
// * State of the actuators.
// * Parameters of the controllers.
class ESAT_ADCSHousekeepingTelemetryPacketClass: public ESAT_ADCSTelemetryPacket
{
  public:
    // Return the ADCS housekeeping telemetry packet identifier.
    byte packetIdentifier();

    // Fill a packet with ADCS housekeeping telemetry.
    void readUserData(ESAT_CCSDSPacket& packet);

  private:
    // Packet identifier of ADCS housekeeping telemetry.
    static const byte PACKET_IDENTIFIER = 0x00;
};

// Public instance of the ADCS housekeeping telemetry packet library.
extern ESAT_ADCSHousekeepingTelemetryPacketClass ESAT_ADCSHousekeepingTelemetryPacket;

#endif /* ESAT_ADCSHousekeepingTelemetryPacket_h */
