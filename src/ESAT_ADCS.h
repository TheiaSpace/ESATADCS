/*
 * Copyright (C) 2017, 2018, 2019 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_ADCS_h
#define ESAT_ADCS_h

#include <Arduino.h>
#include <ESAT_CCSDSPacket.h>
#include <ESAT_CCSDSPacketFromKISSFrameReader.h>
#include <ESAT_CCSDSPacketToKISSFrameWriter.h>
#include <ESAT_SoftwareClock.h>
#include "ESAT_ADCS-measurements/ESAT_AttitudeStateVector.h"
#include "ESAT_ADCS-run-modes/ESAT_ADCSRunMode.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_ADCSTelecommandHandler.h"
#include "ESAT_ADCS-telemetry-packets/ESAT_ADCSTelemetryPacket.h"

// Attitude Determination and Control Subsystem (ADCS) library.
// Use the global instance ESAT_ADCS.
//
// Currently, the ADCS runs as a library used by the On-Board Computer
// program:
//
// * It calls ESAT_ADCS.begin() to start the ADCS.
// * It calls ESAT_ADCS.getApplicationProcessIdentifier() and
//   ESAT_ADCS.handleTelecommand() to respond to telecommands sent to
//   the ADCS.
// * It calls update() to iterate the ADCS loop.
// * It calls telemetryAvailable() and readTelemetry() to retrieve
//   ADCS telemetry.
//
// The ADCS has different run modes.  A run mode is an object that
// implements the ESAT_ADCSRunMode interface.  To change the run mode,
// use ESAT_ADCS.setRunMode().
//
// It is possible to define new ADCS telecommands with new telecommand
// handlers.  A telecommand handler is an object that implements the
// ESAT_ADCSTelecommandHandler interface.  To add new telecommand handler,
// use ESAT_ADCS.registerTelecommandHandler().
class ESAT_ADCSClass
{
  public:
    // Stack a new telemetry packet for emission.
    void addTelemetryPacket(ESAT_ADCSTelemetryPacket& telemetryPacket);

    // Return the current attitude state vector.
    ESAT_AttitudeStateVector attitudeStateVector();

    // Get all ADCS subsystems ready.
    void begin();

    // Disable the reception of telecommands through the USB interface.
    void disableUSBTelecommands();

    // Disable the emission of telemetry from the USB interface.
    void disableUSBTelemetry();

    // Enable the reception of telecommands from the USB interface.
    // Use the buffer for accumulating the partially-received
    // telecommands from one call to readTelecommand() to the next.
    void enableUSBTelecommands(byte buffer[], unsigned long bufferLength);

    // Enable the emission of telemetry through the USB interface.
    void enableUSBTelemetry();

    // Handle a telecommand.
    void handleTelecommand(ESAT_CCSDSPacket& packet);

    // Return the unique identifier of the ADCS.
    word getApplicationProcessIdentifier();

    // Return the period (in seconds) from the previous call to
    // update() to the current call to update().  Attitude control
    // loops may need this.
    float period();

    // Read an incomming telecommand and write it into a packet.
    // Return true if there was a valid telecommand available;
    // otherwise return false.
    boolean readTelecommand(ESAT_CCSDSPacket& packet);

    // Fill a packet with the next ADCS telemetry packet available.
    // Return true if the operation was successful;
    // otherwise return false.
    boolean readTelemetry(ESAT_CCSDSPacket& packet);

    // Register a telecommand handler.
    // On telecommand reception, ESAT_ADCS.handleTelecommand()
    // iterates over the telecommand handlers until it finds one that
    // manages the received telecommand.
    void registerTelecommandHandler(ESAT_ADCSTelecommandHandler& telecommandHandler);

    // Respond to telemetry and telecommand requests coming from the I2C bus.
    // This method does nothing when run on the ESAT OBC board.
    void respondToI2CRequests();

    // Return the current run mode identifier.
    byte runModeIdentifier();

    // Set the ADCS run mode.
    void setRunMode(ESAT_ADCSRunMode& runMode);

    // Set the time of the real-time clock.
    void setTime(ESAT_Timestamp timestamp);

    // Return true if there is a new telemetry packet available.
    boolean telemetryAvailable();

    // Update the ADCS:
    // * Read the sensors.
    // * Perform one iteration.
    // * Make available a new housekeeping telemetry packet.
    void update();

    // Send a telemetry packet through the USB debugging interface.
    void writeTelemetry(ESAT_CCSDSPacket& packet);

  private:
    // Unique identifier of the subsystem.
    static const byte APPLICATION_PROCESS_IDENTIFIER = 2;

    // Version numbers.
    static const byte MAJOR_VERSION_NUMBER = 3;
    static const byte MINOR_VERSION_NUMBER = 4;
    static const byte PATCH_VERSION_NUMBER = 0;

#ifdef ARDUINO_ESAT_ADCS
    // Maximum number of bytes of the telecommand packet data field
    // of telecommand packets coming from the I2C bus.
    static const unsigned long MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH = 1024;

    // Maximum number of bytes of the telemetry packet data field
    // of telemetry packets going out through the I2C bus.
    static const unsigned long MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH = 1024;

    // Back buffer for the packet data field of telecommand packets
    // coming from the I2C bus.
    byte i2cTelecommandPacketData[MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH];

    // Back buffer for the packet data field of telemetry packets
    // going out through the I2C bus.
    byte i2cTelemetryPacketData[MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH];

    // Top element of the stack of telemetry packets for I2C telemetry
    // requests.
    ESAT_ADCSTelemetryPacket* i2cTelemetryPacket;

    // Latest element added to the stack of telemetry packets.
    ESAT_ADCSTelemetryPacket* latestTelemetryPacket;
#endif /* ARDUINO_ESAT_ADCS */

    // Real-time clock.
    // Useful for generating timestamps for telemetry packets.
    ESAT_SoftwareClock clock;

    // Current attitude state vector.
    ESAT_AttitudeStateVector currentAttitudeStateVector;

    // Processor uptime (in milliseconds) at the current call to update().
    unsigned long currentUpdateTime;

    // Processor uptime (in milliseconds) at the previous call to update().
    unsigned long previousUpdateTime;

    // Current run mode.
    ESAT_ADCSRunMode* runMode;

    // First element of the list of telecommand handlers.
    ESAT_ADCSTelecommandHandler* telecommandHandler = nullptr;

    // Top element of the stack of telemetry packets.
    ESAT_ADCSTelemetryPacket* telemetryPacket;

    // Counter of generated telemetry packets.
    word telemetryPacketSequenceCount;

    // Use this to read CCSDS packets from KISS frames coming from the
    // USB interface.
    ESAT_CCSDSPacketFromKISSFrameReader usbReader;

    // Use this to write CCSDS packets in KISS frames to the USB
    // interface.
    ESAT_CCSDSPacketToKISSFrameWriter usbWriter;

    // Add the housekeeping telemetry packet to the telemetry packet stack.
    void addHousekeepingTelemetryPacket();

    // Clear the list of telemetry packets.
    void clearTelemetryPacketList();

    // Fill a telemetry packet with the contents of the
    // given ESAT_ADCSTelemetryPacket.
    // Return true on success; otherwise return false.
    boolean fillTelemetryPacket(ESAT_CCSDSPacket& packet,
                                ESAT_ADCSTelemetryPacket& contents);

    // Return the telemetry packet of given identifier
    // or nullptr if it couldn't be found.
    ESAT_ADCSTelemetryPacket* findTelemetryPacket(byte identifier);

    // Read the sensors needed for attitude determination and control.
    void readSensors();

#ifdef ARDUINO_ESAT_ADCS
    // Respond to a named-packet (of given identifier) telemetry
    // request coming from the I2C bus.
    void respondToNamedPacketTelemetryRequest(byte identifier);

    // Respond to a next-packet telecommand request coming from the
    // I2C bus.
    void respondToNextPacketTelecommandRequest();

    // Respond to a next-packet telemetry request coming from the I2C
    // bus.
    void respondToNextPacketTelemetryRequest();
#endif /* ARDUINO_ESAT_ADCS */

    // Actuate according to the current run mode.
    void run();

    // Recalculate the period.
    void updatePeriod();
};

// Global instance of the ADCS library.
extern ESAT_ADCSClass ESAT_ADCS;

#endif /* ESAT_ADCS_h */
