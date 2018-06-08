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
#ifdef ARDUINO_ESAT_ADCS
#include <ESAT_Util.h>
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
#include <ESAT_CCSDSPacket.h>
#include <ESAT_I2CMaster.h>
#endif /* ARDUINO_ESAT_OBC */
#include <Wire.h>

void ESAT_WheelClass::begin()
{
#ifdef ARDUINO_ESAT_ADCS
  pinMode(EN5V, OUTPUT);
  calibrateElectronicSpeedController();
  writeDutyCycle(0);
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  pinMode(PWM, OUTPUT);
  writeDutyCycle(0);
  electronicSpeedController.attach(PWM);
  calibrateElectronicSpeedController();
#endif /* ARDUINO_ESAT_OBC */
}

void ESAT_WheelClass::calibrateElectronicSpeedController()
{
#ifdef ARDUINO_ESAT_ADCS
  switchElectronicSpeedController(false);
  delay(1000);
  switchElectronicSpeedController(true);
  delay(1000);
  Wire1.beginTransmission(ELECTRONIC_SPEED_CONTROLLER_ADDRESS);
  Wire1.write(WHEEL_SPEED_REGISTER);
  Wire1.write(0xFF);
  Wire1.write(0xFF);
  (void) Wire1.endTransmission();
  Wire1.beginTransmission(ELECTRONIC_SPEED_CONTROLLER_ADDRESS);
  Wire1.write(WHEEL_SPEED_REGISTER);
  Wire1.write(0x00);
  Wire1.write(0x00);
  (void) Wire1.endTransmission();
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  // Perform the ESC calibration sequence (high, low and medium again).
  switchElectronicSpeedController(false);
  delay(1000);
  writeDutyCycle(100);
  switchElectronicSpeedController(true);
  delay(2000);
  writeDutyCycle(-100);
  delay(1000);
  writeDutyCycle(0);
  delay(1000);
#endif /* ARDUINO_ESAT_OBC */
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

void ESAT_WheelClass::switchElectronicSpeedController(boolean on)
{
#ifdef ARDUINO_ESAT_ADCS
  if (on)
  {
    digitalWrite(EN5V, HIGH);
  }
  else
  {
    digitalWrite(EN5V, LOW);
  }
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
  packet.writeBoolean(on);
  ESAT_I2CMaster.writeTelecommand(Wire,
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
#ifdef ARDUINO_ESAT_ADCS
  dutyCycle = constrainDutyCycle(newDutyCycle);
  const int integerDutyCycle = map(dutyCycle, -100, 100, -32768, 32767);
  const word dutyCycleBytes = ESAT_Util.intToWord(integerDutyCycle);
  Wire1.beginTransmission(ELECTRONIC_SPEED_CONTROLLER_ADDRESS);
  (void) Wire1.write(WHEEL_SPEED_REGISTER);
  (void) Wire1.write(highByte(dutyCycleBytes));
  (void) Wire1.write(lowByte(dutyCycleBytes));
  (void) Wire1.endTransmission();
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  dutyCycle = constrainDutyCycle(newDutyCycle);
  const word microseconds =
    (MAXIMUM_PULSE_WIDTH + MINIMUM_PULSE_WIDTH) / 2
    + (MAXIMUM_PULSE_WIDTH - MINIMUM_PULSE_WIDTH) / 2
      * dutyCycle / 100;
  electronicSpeedController.writeMicroseconds(microseconds);
#endif /* ARDUINO_ESAT_OBC */
}

ESAT_WheelClass ESAT_Wheel;
