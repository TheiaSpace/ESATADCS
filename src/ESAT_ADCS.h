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

#ifndef ESAT_ADCS_h
#define ESAT_ADCS_h

#include <Arduino.h>
#include <ESAT_CCSDSPacket.h>
#include "ESAT_ADCS-measurements/ESAT_AttitudeStateVector.h"
#include "ESAT_ADCS-run-modes/ESAT_ADCSRunMode.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_ADCSTelecommandHandler.h"

// Attitude determination and control subsystem library.
// Use the global instance ESAT_ADCS.
class ESAT_ADCSClass
{
  public:
    // Get all ADCS subsystems ready.
    // Set the control cycle period (in milliseconds).
    void begin(word period);

    // Handle a telecommand.
    void handleTelecommand(ESAT_CCSDSPacket& packet);

    // Return the unique identifier of the ADCS.
    word getApplicationProcessIdentifier();

    // Fill a packet with the next ADCS telemetry packet available.
    // Return true if the operation was successful;
    // otherwise return false.
    boolean readTelemetry(ESAT_CCSDSPacket& packet);

    // Register a telecommand handler.
    // On telecommand reception, ESAT_ADCS.handleTelecommand()
    // iterates over the telecommand handlers until it finds one that
    // manages the received telecommand.
    void registerTelecommandHandler(ESAT_ADCSTelecommandHandler& telecommandHandler);

    // Set the ADCS run mode.
    void setRunMode(ESAT_ADCSRunMode& runMode);

    // Return true if there is a new telemetry packet available.
    boolean telemetryAvailable();

    // Update the ADCS:
    // * Read the sensors.
    // * Perform one iteration.
    // * Make available a new telemetry packet.
    void update();

  private:
    // Telemetry packet identifiers.
    enum TelemetryPacketIdentifier
    {
      HOUSEKEEPING = 0,
    };

    // Unique identifier of the subsystem.
    static const byte APPLICATION_PROCESS_IDENTIFIER = 2;

    // Version numbers.
    static const byte MAJOR_VERSION_NUMBER = 3;
    static const byte MINOR_VERSION_NUMBER = 0;
    static const byte PATCH_VERSION_NUMBER = 0;

    // Maximum number of telecommand handlers.
    static const byte MAXIMUM_NUMBER_OF_TELECOMMAND_HANDLERS = 16;

    // Current attitude state vector.
    ESAT_AttitudeStateVector attitudeStateVector;

    // True when a new telemetry packet is available; false otherwise.
    boolean newTelemetryPacket;

    // Number of registered telecommand handlers.
    byte numberOfTelecommandHandlers;

    // Current run mode.
    ESAT_ADCSRunMode* runMode;

    // List of telecommand handlers.
    ESAT_ADCSTelecommandHandler* telecommandHandlers[MAXIMUM_NUMBER_OF_TELECOMMAND_HANDLERS];

    // Counter of generated telemetry packets.
    word telemetryPacketSequenceCount;

    // Read the sensors needed for attitude determination and control.
    void readSensors();

    // Actuate according to the current run mode.
    void run();
};

// Global instance of the ADCS library.
extern ESAT_ADCSClass ESAT_ADCS;

#endif /* ESAT_ADCS_h */
