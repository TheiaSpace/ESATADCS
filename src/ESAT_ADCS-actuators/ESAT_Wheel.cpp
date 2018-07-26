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

#include "ESAT_ADCS-actuators/ESAT_Wheel.h"
#ifdef ARDUINO_ESAT_OBC
#include <ESAT_CCSDSPacket.h>
#include <ESAT_I2CMaster.h>
#include <Wire.h>
#endif /* ARDUINO_ESAT_OBC */

void ESAT_WheelClass::begin()
{
#ifdef ARDUINO_ESAT_ADCS
  pinMode(EN5V, OUTPUT);
  electronicSpeedController.attach(PWM_A);
  calibrateElectronicSpeedController();
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  electronicSpeedController.attach(PWM);
  calibrateElectronicSpeedController();
#endif /* ARDUINO_ESAT_OBC */
}

void ESAT_WheelClass::calibrateElectronicSpeedController()
{
  // Perform the ESC calibration sequence (high, low and medium again).
  switchOffElectronicSpeedController();
  delay(1000);
  writeDutyCycle(100);
  switchOnElectronicSpeedController();
  delay(2000);
  writeDutyCycle(-100);
  delay(1000);
  writeDutyCycle(0);
  delay(1000);
}

float ESAT_WheelClass::constrainDutyCycle(const float dutyCycle)
{
  if (dutyCycle > 100)
  {
    return 100;
  }
  if (dutyCycle < -100)
  {
    return -100;
  }
  return dutyCycle;
}

word ESAT_WheelClass::constrainSpeed(const int wheelSpeed)
{
  return constrain(wheelSpeed, -MAXIMUM_WHEEL_SPEED, MAXIMUM_WHEEL_SPEED);
}

float ESAT_WheelClass::readDutyCycle()
{
  return dutyCycle;
}

void ESAT_WheelClass::switchOffElectronicSpeedController()
{
#ifdef ARDUINO_ESAT_ADCS
  digitalWrite(EN5V, LOW);
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  const byte packetDataBufferLength = ESAT_CCSDSSecondaryHeader::LENGTH + 1;
  byte buffer[packetDataBufferLength];
  ESAT_CCSDSPacket packet(buffer, packetDataBufferLength);
  packet.flush();
  ESAT_CCSDSPrimaryHeader primaryHeader;
  primaryHeader.packetVersionNumber = 0;
  primaryHeader.packetType = primaryHeader.TELECOMMAND;
  primaryHeader.secondaryHeaderFlag = primaryHeader.SECONDARY_HEADER_IS_PRESENT;
  primaryHeader.applicationProcessIdentifier = POWER_LINE_IDENTIFIER;
  primaryHeader.sequenceFlags = primaryHeader.UNSEGMENTED_USER_DATA;
  primaryHeader.packetSequenceCount = 0;
  packet.writePrimaryHeader(primaryHeader);
  ESAT_CCSDSSecondaryHeader secondaryHeader;
  secondaryHeader.majorVersionNumber = POWER_LINE_MAJOR_VERSION_NUMBER;
  secondaryHeader.minorVersionNumber = POWER_LINE_MINOR_VERSION_NUMBER;
  secondaryHeader.patchVersionNumber = POWER_LINE_PATCH_VERSION_NUMBER;
  secondaryHeader.packetIdentifier = POWER_LINE_COMMAND_CODE;
  packet.writeSecondaryHeader(secondaryHeader);
  packet.writeByte(POWER_LINE_SWITCH_OFF);
  ESAT_I2CMaster.writePacket(Wire,
                             POWER_LINE_ADDRESS,
                             packet,
                             POWER_LINE_MILLISECONDS_AFTER_WRITES,
                             POWER_LINE_ATTEMPTS,
                             POWER_LINE_MILLISECONDS_BETWEEN_ATTEMPTS);
#endif /* ARDUINO_ESAT_OBC */
}

void ESAT_WheelClass::switchOnElectronicSpeedController()
{
#ifdef ARDUINO_ESAT_ADCS
  digitalWrite(EN5V, HIGH);
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  const byte packetDataBufferLength = ESAT_CCSDSSecondaryHeader::LENGTH + 1;
  byte buffer[packetDataBufferLength];
  ESAT_CCSDSPacket packet(buffer, packetDataBufferLength);
  packet.flush();
  ESAT_CCSDSPrimaryHeader primaryHeader;
  primaryHeader.packetVersionNumber = 0;
  primaryHeader.packetType = primaryHeader.TELECOMMAND;
  primaryHeader.secondaryHeaderFlag = primaryHeader.SECONDARY_HEADER_IS_PRESENT;
  primaryHeader.applicationProcessIdentifier = POWER_LINE_IDENTIFIER;
  primaryHeader.sequenceFlags = primaryHeader.UNSEGMENTED_USER_DATA;
  primaryHeader.packetSequenceCount = 0;
  packet.writePrimaryHeader(primaryHeader);
  ESAT_CCSDSSecondaryHeader secondaryHeader;
  secondaryHeader.majorVersionNumber = POWER_LINE_MAJOR_VERSION_NUMBER;
  secondaryHeader.minorVersionNumber = POWER_LINE_MINOR_VERSION_NUMBER;
  secondaryHeader.patchVersionNumber = POWER_LINE_PATCH_VERSION_NUMBER;
  secondaryHeader.packetIdentifier = POWER_LINE_COMMAND_CODE;
  packet.writeSecondaryHeader(secondaryHeader);
  packet.writeByte(POWER_LINE_SWITCH_ON);
  ESAT_I2CMaster.writePacket(Wire,
                             POWER_LINE_ADDRESS,
                             packet,
                             POWER_LINE_MILLISECONDS_AFTER_WRITES,
                             POWER_LINE_ATTEMPTS,
                             POWER_LINE_MILLISECONDS_BETWEEN_ATTEMPTS);
#endif /* ARDUINO_ESAT_OBC */
}

void ESAT_WheelClass::writeSpeed(const int rpm)
{
  writeDutyCycle(DUTY_CYCLE_PER_RPM * constrainSpeed(rpm));
}

void ESAT_WheelClass::writeDutyCycle(const float newDutyCycle)
{
  dutyCycle = constrainDutyCycle(newDutyCycle);
  const word microseconds =
    (MAXIMUM_PULSE_WIDTH + MINIMUM_PULSE_WIDTH) / 2
    + (MAXIMUM_PULSE_WIDTH - MINIMUM_PULSE_WIDTH) / 2
      * dutyCycle / 100;
  electronicSpeedController.writeMicroseconds(microseconds);
}

ESAT_WheelClass ESAT_Wheel;
