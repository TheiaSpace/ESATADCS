/*
 * Copyright (C) 2017-2018 Theia Space, Universidad Politécnica de Madrid
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
 * along with Theia Space's ESAT EPS library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <ESAT_ADCS.h>
#include <ESAT_Timer.h>
#ifdef ARDUINO_ESAT_OBC
#include <USBSerial.h>
#endif /* ARDUINO_ESAT_OBC */
#include <Wire.h>

// Main program of the ADCS board: respond to telecommands,
// perform the actions of the attitude determination and control
// loop, and send telemetry.

// After a call to ADCS.begin(), the board will respond to I2C messages
// on the Wire (SCL_O, SDA_O) interface through the interrupt handling
// code of the ESAT_I2CSlave module (part of the ESATUtil library): it
// will send telemetry when requested and it will queue telecommands
// for later retrieval on the main loop with readTelecommand().

// Target period (in milliseconds) for main loop activities with the
// exception of the response to I2C requests.
constexpr word TARGET_PERIOD = 1000;

// Maximum packet data length we will handle.
const word PACKET_DATA_BUFFER_LENGTH = 1024;

// Store the packet data field of telecommands coming from the USB
// interface in this buffer.
byte telecommandPacketData[PACKET_DATA_BUFFER_LENGTH];

// Start the peripherals and do some initial bookkeeping work.
void setup()
{
#ifdef ARDUINO_ESAT_ADCS
  Serial.begin(9600);
  Serial.blockOnOverrun(false);
  Wire.begin(ESAT_ADCS.getApplicationProcessIdentifier());
  Wire1.begin();
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  USB.begin();
  Wire.begin();
#endif /* ARDUINO_ESAT_OBC */
  ESAT_ADCS.enableUSBTelemetry();
  ESAT_ADCS.enableUSBTelecommands(telecommandPacketData,
                                  sizeof(telecommandPacketData));
  ESAT_ADCS.begin();
  ESAT_Timer.begin(TARGET_PERIOD);
}

// Body of the main loop of the program:
// - Retrieve the incomming telecommands (from the I2C interface
//   and from the USB interface).
// - Handle the incoming telecommands.
// - Perform one iteration of the attitude determination and control
//   main loop.
// - Retrieve the telemetry packets.
// - Write the telemetry packets through the USB interface.
// The packet data buffer used for telecommand and telemetry
// operations must be big enough for holding all the packet
// data; if the buffer is too small, the packets will be dropped.
void loop()
{
  if (ESAT_Timer.ellapsedMilliseconds() >= TARGET_PERIOD)
  {
    byte buffer[PACKET_DATA_BUFFER_LENGTH];
    ESAT_CCSDSPacket packet(buffer, sizeof(buffer));
    while (ESAT_ADCS.readTelecommand(packet))
    {
      ESAT_ADCS.handleTelecommand(packet);
    }
    ESAT_ADCS.update();
    while (ESAT_ADCS.readTelemetry(packet))
    {
      ESAT_ADCS.writeTelemetry(packet);
    }
    ESAT_Timer.begin(TARGET_PERIOD);
  }
#ifdef ARDUINO_ESAT_ADCS
  ESAT_ADCS.respondToI2CRequests();
#endif /* ARDUINO_ESAT_ADCS */
}
