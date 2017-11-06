/*
 *       ESAT ADCS library.
 *       Author Daniel Calvo @ Theia
 ******
 * Angles are defined as follows
 *
 *             X-(180º)
 *           __________
 *          |     --> Y||
 * Y-(270º) |    |     || panel  Y+(90º)
 *          |    V X   ||
 *          |__________||
 *           ----------
 *             panel
 *             X+(0º)
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

#include "ESAT_ADCS.h"
#include "ESAT_AttitudePIDController.h"
#include "ESAT_AttitudeTelecommandHandler.h"
#include "ESAT_CoarseSunSensor.h"
#include "ESAT_FollowMagneticAngleRunMode.h"
#include "ESAT_FollowSunAngleRunMode.h"
#include "ESAT_Gyroscope.h"
#include "ESAT_Magnetometer.h"
#include "ESAT_MagnetorquerApplyMaximumTorqueRunMode.h"
#include "ESAT_MagnetorquerDemagnetizeRunMode.h"
#include "ESAT_MagnetorquerEnableRunMode.h"
#include "ESAT_Magnetorquer.h"
#include "ESAT_MagnetorquerSetXPolarityRunMode.h"
#include "ESAT_MagnetorquerSetYPolarityRunMode.h"
#include "ESAT_MagnetorquerTelecommandHandler.h"
#include "ESAT_OBCClock.h"
#include "ESAT_StopActuatorsRunMode.h"
#include "ESAT_StopActuatorsTelecommandHandler.h"
#include "ESAT_Tachometer.h"
#include "ESAT_Timestamp.h"
#include "ESAT_Wheel.h"
#include "ESAT_WheelPIDController.h"
#include "ESAT_WheelSetDutyCycleRunMode.h"
#include "ESAT_WheelSetSpeedRunMode.h"
#include "ESAT_WheelTelecommandHandler.h"

void ESAT_ADCSClass::begin(const word period)
{
  newTelemetryPacket = false;
  telemetryPacketSequenceCount = 0;
  ESAT_AttitudePIDController.begin(period / 1000.);
  ESAT_Wheel.begin();
  ESAT_WheelPIDController.begin(period / 1000.);
  ESAT_Gyroscope.begin(ESAT_Gyroscope.FULL_SCALE_2000_DEGREES_PER_SECOND);
  ESAT_Magnetometer.begin();
  ESAT_CoarseSunSensor.begin();
  ESAT_Tachometer.begin();
  ESAT_Magnetorquer.begin();
  setRunMode(ESAT_StopActuatorsRunMode);
  numberOfTelecommandHandlers = 0;
  registerTelecommandHandler(ESAT_AttitudeTelecommandHandler);
  registerTelecommandHandler(ESAT_WheelTelecommandHandler);
  registerTelecommandHandler(ESAT_MagnetorquerTelecommandHandler);
  registerTelecommandHandler(ESAT_StopActuatorsTelecommandHandler);
}

word ESAT_ADCSClass::getApplicationProcessIdentifier()
{
  return APPLICATION_PROCESS_IDENTIFIER;
}

void ESAT_ADCSClass::handleTelecommand(ESAT_CCSDSPacket& packet)
{
  packet.rewind();
  const ESAT_CCSDSPrimaryHeader primaryHeader = packet.readPrimaryHeader();
  if (primaryHeader.applicationProcessIdentifier
      != getApplicationProcessIdentifier())
  {
    return;
  }
  if (primaryHeader.packetType != primaryHeader.TELECOMMAND)
  {
    return;
  }
  if (primaryHeader.packetDataLength < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return;
  }
  for (int i = 0; i < numberOfTelecommandHandlers; i++)
  {
    const boolean handled = telecommandHandlers[i]->handleTelecommand(packet);
    if (handled)
    {
      return;
    }
  }
}

void ESAT_ADCSClass::readSensors()
{
  attitudeStateVector.wheelSpeed = ESAT_Tachometer.read();
  const boolean enableMagnetorquerDriver = ESAT_Magnetorquer.readEnable();
  ESAT_Magnetorquer.writeEnable(false);
  delay(20);
  ESAT_Gyroscope.error = false;
  attitudeStateVector.rotationalSpeed = ESAT_Gyroscope.read(3);
  ESAT_Magnetometer.error = false;
  attitudeStateVector.magneticAngle = ESAT_Magnetometer.read();
  ESAT_Magnetorquer.writeEnable(enableMagnetorquerDriver);
  attitudeStateVector.sunAngle = ESAT_CoarseSunSensor.read();
}

boolean ESAT_ADCSClass::readTelemetry(ESAT_CCSDSPacket& packet)
{
  if (!newTelemetryPacket)
  {
    return false;
  }
  newTelemetryPacket = false;
  if (packet.capacity() < HOUSEKEEPING_TELEMETRY_PACKET_LENGTH)
  {
    return false;
  }
  packet.clear();
  // Primary header.
  ESAT_CCSDSPrimaryHeader primaryHeader;
  primaryHeader.packetVersionNumber = 0;
  primaryHeader.packetType =
    primaryHeader.TELEMETRY;
  primaryHeader.secondaryHeaderFlag =
    primaryHeader.SECONDARY_HEADER_IS_PRESENT;
  primaryHeader.applicationProcessIdentifier =
    getApplicationProcessIdentifier();
  primaryHeader.sequenceFlags =
    primaryHeader.UNSEGMENTED_USER_DATA;
  primaryHeader.packetSequenceCount =
    telemetryPacketSequenceCount;
  packet.writePrimaryHeader(primaryHeader);
  // Secondary header.
  ESAT_CCSDSSecondaryHeader secondaryHeader;
  secondaryHeader.preamble =
    secondaryHeader.CALENDAR_SEGMENTED_TIME_CODE_MONTH_DAY_VARIANT_1_SECOND_RESOLUTION;
  secondaryHeader.timestamp = ESAT_OBCClock.read();
  secondaryHeader.majorVersionNumber = MAJOR_VERSION_NUMBER;
  secondaryHeader.minorVersionNumber = MINOR_VERSION_NUMBER;
  secondaryHeader.patchVersionNumber = PATCH_VERSION_NUMBER;
  secondaryHeader.packetIdentifier = HOUSEKEEPING;
  packet.writeSecondaryHeader(secondaryHeader);
  // User data.
  packet.writeByte(runMode->identifier());
  packet.writeWord(ESAT_AttitudePIDController.targetAngle);
  packet.writeWord(attitudeStateVector.magneticAngle);
  packet.writeWord(attitudeStateVector.sunAngle);
  packet.writeWord(attitudeStateVector.rotationalSpeed);
  packet.writeFloat(ESAT_AttitudePIDController.proportionalGain);
  packet.writeFloat(ESAT_AttitudePIDController.integralGain);
  packet.writeFloat(ESAT_AttitudePIDController.derivativeGain);
  packet.writeBoolean(ESAT_AttitudePIDController.useGyroscope);
  packet.writeByte(ESAT_AttitudePIDController.actuator);
  packet.writeFloat(ESAT_Wheel.readDutyCycle());
  packet.writeWord(attitudeStateVector.wheelSpeed);
  packet.writeFloat(ESAT_WheelPIDController.proportionalGain);
  packet.writeFloat(ESAT_WheelPIDController.integralGain);
  packet.writeFloat(ESAT_WheelPIDController.derivativeGain);
  packet.writeByte(ESAT_Magnetorquer.readEnable());
  packet.writeByte(byte(ESAT_Magnetorquer.readX()));
  packet.writeByte(byte(ESAT_Magnetorquer.readY()));
  packet.writeBoolean(ESAT_Gyroscope.error);
  packet.writeBoolean(ESAT_Magnetometer.error);
  // End of user data.
  packet.flush();
  telemetryPacketSequenceCount = telemetryPacketSequenceCount + 1;
  return true;
}

void ESAT_ADCSClass::registerTelecommandHandler(ESAT_ADCSTelecommandHandler& telecommandHandler)
{
  if (numberOfTelecommandHandlers == MAXIMUM_NUMBER_OF_TELECOMMAND_HANDLERS)
  {
    return;
  }
  telecommandHandlers[numberOfTelecommandHandlers] = &telecommandHandler;
  numberOfTelecommandHandlers = numberOfTelecommandHandlers + 1;
}

void ESAT_ADCSClass::run()
{
  runMode->loop(attitudeStateVector);
}

void ESAT_ADCSClass::setRunMode(ESAT_ADCSRunMode& newRunMode)
{
  runMode = &newRunMode;
}

boolean ESAT_ADCSClass::telemetryAvailable()
{
  return newTelemetryPacket;
}

void ESAT_ADCSClass::update()
{
  readSensors();
  run();
  newTelemetryPacket = true;
}

ESAT_ADCSClass ESAT_ADCS;
