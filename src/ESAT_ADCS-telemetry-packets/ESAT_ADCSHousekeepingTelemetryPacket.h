/*
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

// ADCS housekeeping telemetry:
// * Mode of operation.
// * Attitude state vector.
// * State of the actuators.
// * Parameters of the controllers.
class ESAT_ADCSHousekeepingTelemetryPacket: public ESAT_ADCSTelemetryPacket
{
  public:
    // Default constructor.
    ESAT_ADCSHousekeepingTelemetryPacket();

    // Instantiate a new ADCS housekeeping telemetry packet with a
    // given attitude state vector and run mode.
    ESAT_ADCSHousekeepingTelemetryPacket(ESAT_AttitudeStateVector attitudeStateVector,
                                         byte runModeIdentifier);

    // Return the ADCS housekeeping telemetry packet identifier.
    byte packetIdentifier();

    // Fill a packet with ADCS housekeeping telemetry.
    void readUserData(ESAT_CCSDSPacket& packet);

  private:
    // Packet identifier of ADCS housekeeping telemetry.
    static const byte PACKET_IDENTIFIER = 0x00;

    // Attitude state vector.
    ESAT_AttitudeStateVector attitudeStateVector;

    // Run mode identifier.
    byte runModeIdentifier;
};

#endif /* ESAT_ADCSHousekeepingTelemetryPacket_h */
