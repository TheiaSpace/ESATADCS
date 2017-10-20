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

#include "ESAT_Wheel.h"
#include <ESAT_CCSDSPacket.h>
#include <ESAT_I2CMaster.h>
#include <Wire.h>

void ESAT_WheelClass::begin()
{
  calibration[0] = 1.28e2;
  calibration[1] = 1.90e-2;
  pinMode(PIN, OUTPUT);
  electronicSpeedController.attach(PIN);
  delay(1000);
  programElectronicSpeedController();
}

void ESAT_WheelClass::write(const word rpm)
{
  const float unconstrainedDutyCycle =
    calibration[0] + rpm * calibration[1];
  const byte dutyCycle =
          constrain(round(unconstrainedDutyCycle), 0, 255);
  writeDutyCycle(dutyCycle);
}

void ESAT_WheelClass::writeDutyCycle(byte dutyCycle)
{
  const unsigned int microseconds = map(dutyCycle, 0, 255, MINIMUM, MAXIMUM);
  electronicSpeedController.writeMicroseconds(microseconds);
}

void ESAT_WheelClass::programElectronicSpeedController()
{
  // Perform the ESC programming sequence (high, low and medium again)
  switchElectronicSpeedController(false);
  delay(1000);
  writeDutyCycle(255);
  switchElectronicSpeedController(true);
  delay(2000);
  writeDutyCycle(0);
  delay(1000);
  writeDutyCycle(128);
  delay(1000);
}

void ESAT_WheelClass::switchElectronicSpeedController(boolean on)
{
  const byte packetDataBufferLength = ESAT_CCSDSSecondaryHeader::LENGTH + 1;
  byte buffer[packetDataBufferLength];
  ESAT_CCSDSPacket packet(buffer, packetDataBufferLength);
  packet.clear();
  packet.writePacketVersionNumber(0);
  packet.writePacketType(packet.TELECOMMAND);
  packet.writeSecondaryHeaderFlag(packet.SECONDARY_HEADER_IS_PRESENT);
  packet.writeApplicationProcessIdentifier(POWER_LINE_IDENTIFIER);
  packet.writeSequenceFlags(packet.UNSEGMENTED_USER_DATA);
  packet.writePacketSequenceCount(0);
  ESAT_CCSDSSecondaryHeader secondaryHeader;
  secondaryHeader.majorVersionNumber = POWER_LINE_MAJOR_VERSION_NUMBER;
  secondaryHeader.minorVersionNumber = POWER_LINE_MINOR_VERSION_NUMBER;
  secondaryHeader.patchVersionNumber = POWER_LINE_PATCH_VERSION_NUMBER;
  secondaryHeader.packetIdentifier = POWER_LINE_COMMAND_CODE;
  packet.writeSecondaryHeader(secondaryHeader);
  packet.writeBoolean(on);
  packet.updatePacketDataLength();
  ESAT_I2CMaster.writeTelecommand(Wire,
                                  POWER_LINE_ADDRESS,
                                  packet,
                                  POWER_LINE_MILLISECONDS_AFTER_WRITES,
                                  POWER_LINE_TRIES,
                                  POWER_LINE_MILLISECONDS_BETWEEN_RETRIES);
}

ESAT_WheelClass ESAT_Wheel;
