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

#ifndef ESAT_ADCSTelemetryPacket_h
#define ESAT_ADCSTelemetryPacket_h

#include <Arduino.h>
#include <ESAT_CCSDSPacket.h>

// Implement this interface to generate ADCS telemetry packets.  On
// each ADCS update cycle, stack the packets that will generated by
// the next calls to ESAT_ADCS.readTelemetry() with
// ESAT_ADCS.addTelemetryPacket().  ESAT_ADCS.readTelemetry() will
// remove ADCS telemetry packet objects from the stack and call their
// readUserData() methods to fill the packet data fields of the
// packets.
class ESAT_ADCSTelemetryPacket
{
  public:
    virtual ~ESAT_ADCSTelemetryPacket() {};

    // Return the packet identifier.  Each telemetry packet type has
    // its own unique identifier number.
    virtual byte packetIdentifier() = 0;

    // Fill the user data field of a packet with telemetry.
    // The primary header and secondary header are already filled.
    virtual void readUserData(ESAT_CCSDSPacket& packet) = 0;
};

#endif /* ESAT_ADCSTelemetryPacket_h */
