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
#include "ESAT_OBCClock.h"
#include "ESAT_StopActuatorsRunMode.h"
#include "ESAT_Tachometer.h"
#include "ESAT_Timestamp.h"
#include "ESAT_Wheel.h"
#include "ESAT_WheelPIDController.h"
#include "ESAT_WheelSetDutyCycleRunMode.h"
#include "ESAT_WheelSetSpeedRunMode.h"

void ESAT_ADCSClass::begin(const word period)
{
  newTelemetryPacket = false;
  runCode = STOP_ACTUATORS;
  telemetryPacketSequenceCount = 0;
  ESAT_AttitudePIDController.begin(period / 1000.);
  ESAT_Wheel.begin();
  ESAT_WheelPIDController.begin(period / 1000.);
  ESAT_Gyroscope.begin(ESAT_Gyroscope.FULL_SCALE_2000_DEGREES_PER_SECOND);
  ESAT_Magnetometer.begin();
  ESAT_CoarseSunSensor.begin();
  ESAT_Tachometer.begin();
  ESAT_Magnetorquer.begin();
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
  if (primaryHeader.packetDataLength < MINIMUM_COMMAND_PAYLOAD_DATA_LENGTH)
  {
    return;
  }
  const ESAT_CCSDSSecondaryHeader secondaryHeader = packet.readSecondaryHeader();
  if (secondaryHeader.majorVersionNumber < MAJOR_VERSION_NUMBER)
  {
    return;
  }
  switch (secondaryHeader.packetIdentifier)
  {
    case FOLLOW_MAGNETIC_ANGLE_COMMAND:
      handleFollowMagneticAngleCommand(packet);
      break;
    case FOLLOW_SUN_ANGLE_COMMAND:
      handleFollowSunAngleCommand(packet);
      break;
    case ATTITUDE_CONTROLLER_SET_PROPORTIONAL_GAIN_COMMAND:
      handleAttitudeControllerSetProportionalGainCommand(packet);
      break;
    case ATTITUDE_CONTROLLER_SET_INTEGRAL_GAIN_COMMAND:
      handleAttitudeControllerSetIntegralGainCommand(packet);
      break;
    case ATTITUDE_CONTROLLER_SET_DERIVATIVE_GAIN_COMMAND:
      handleAttitudeControllerSetDerivativeGainCommand(packet);
      break;
    case ATTITUDE_CONTROLLER_USE_GYROSCOPE_COMMAND:
      handleAttitudeControllerUseGyroscopeCommand(packet);
      break;
    case ATTITUDE_CONTROLLER_SET_ACTUATOR_COMMAND:
      handleAttitudeControllerSetActuatorCommand(packet);
      break;
    case ATTITUDE_CONTROLLER_SET_DEADBAND_COMMAND:
      handleAttitudeControllerSetDeadbandCommand(packet);
      break;
    case ATTITUDE_CONTROLLER_SET_DETUMBLING_THRESHOLD_COMMAND:
      handleAttitudeControllerSetDetumblingThresholdCommand(packet);
      break;
    case WHEEL_SET_DUTY_CYCLE_COMMAND:
      handleWheelSetDutyCycleCommand(packet);
      break;
    case WHEEL_SET_SPEED_COMMAND:
      handleWheelSetSpeedCommand(packet);
      break;
    case WHEEL_CONTROLLER_SET_PROPORTIONAL_GAIN_COMMAND:
      handleWheelControllerSetProportionalGainCommand(packet);
      break;
    case WHEEL_CONTROLLER_SET_INTEGRAL_GAIN_COMMAND:
      handleWheelControllerSetIntegralGainCommand(packet);
      break;
    case WHEEL_CONTROLLER_SET_DERIVATIVE_GAIN_COMMAND:
      handleWheelControllerSetDerivativeGainCommand(packet);
      break;
    case WHEEL_CONTROLLER_RESET_SPEED_ERROR_INTEGRAL:
      handleWheelControllerResetSpeedErrorIntegral(packet);
      break;
    case MAGNETORQUER_ENABLE_COMMAND:
      handleMagnetorquerEnableCommand(packet);
      break;
    case MAGNETORQUER_SET_X_POLARITY_COMMAND:
      handleMagnetorquerSetXPolarityCommand(packet);
      break;
    case MAGNETORQUER_SET_Y_POLARITY_COMMAND:
      handleMagnetorquerSetYPolarityCommand(packet);
      break;
    case MAGNETORQUER_APPLY_MAXIMUM_TORQUE_COMMAND:
      handleMagnetorquerApplyMaximumTorqueCommand(packet);
      break;
    case MAGNETORQUER_DEMAGNETIZE_COMMAND:
      handleMagnetorquerDemagnetizeCommand(packet);
      break;
    case STOP_ACTUATORS_COMMAND:
      handleStopActuatorsCommand(packet);
    default:
      break;
  }
}

void ESAT_ADCSClass::handleFollowMagneticAngleCommand(ESAT_CCSDSPacket& packet)
{
  runCode = FOLLOW_MAGNETIC_ANGLE;
  const word rawTargetAttitude = packet.readWord();
  ESAT_AttitudePIDController.targetAngle = rawTargetAttitude % 360;
}

void ESAT_ADCSClass::handleFollowSunAngleCommand(ESAT_CCSDSPacket& packet)
{
  runCode = FOLLOW_SUN_ANGLE;
  const word rawTargetAttitude = packet.readWord();
  ESAT_AttitudePIDController.targetAngle = rawTargetAttitude % 360;
}

void ESAT_ADCSClass::handleAttitudeControllerSetProportionalGainCommand(ESAT_CCSDSPacket& packet)
{
  ESAT_AttitudePIDController.proportionalGain = packet.readFloat();
}

void ESAT_ADCSClass::handleAttitudeControllerSetIntegralGainCommand(ESAT_CCSDSPacket& packet)
{
  ESAT_AttitudePIDController.integralGain = packet.readFloat();
}

void ESAT_ADCSClass::handleAttitudeControllerSetDerivativeGainCommand(ESAT_CCSDSPacket& packet)
{
  ESAT_AttitudePIDController.derivativeGain = packet.readFloat();
}

void ESAT_ADCSClass::handleAttitudeControllerUseGyroscopeCommand(ESAT_CCSDSPacket& packet)
{
  ESAT_AttitudePIDController.useGyroscope = packet.readBoolean();
}

void ESAT_ADCSClass::handleAttitudeControllerSetActuatorCommand(ESAT_CCSDSPacket& packet)
{
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    ESAT_AttitudePIDController.actuator =
      ESAT_AttitudePIDController.WHEEL;
  }
  else
  {
    ESAT_AttitudePIDController.actuator =
      ESAT_AttitudePIDController.MAGNETORQUER;
  }
}

void ESAT_ADCSClass::handleAttitudeControllerSetDeadbandCommand(ESAT_CCSDSPacket& packet)
{
  ESAT_AttitudePIDController.errorDeadband = packet.readWord();
  ESAT_AttitudePIDController.errorDerivativeDeadband = packet.readWord();
}

void ESAT_ADCSClass::handleAttitudeControllerSetDetumblingThresholdCommand(ESAT_CCSDSPacket& packet)

{
  ESAT_AttitudePIDController.detumblingThreshold = packet.readWord();
}

void ESAT_ADCSClass::handleWheelSetDutyCycleCommand(ESAT_CCSDSPacket& packet)
{
  runCode = WHEEL_SET_DUTY_CYCLE;
  ESAT_WheelSetDutyCycleRunMode.dutyCycle = packet.readFloat();
}

void ESAT_ADCSClass::handleWheelSetSpeedCommand(ESAT_CCSDSPacket& packet)
{
  runCode = WHEEL_SET_SPEED;
  ESAT_WheelSetSpeedRunMode.targetSpeed = packet.readWord();
}

void ESAT_ADCSClass::handleWheelControllerSetProportionalGainCommand(ESAT_CCSDSPacket& packet)
{
  ESAT_WheelPIDController.proportionalGain = packet.readFloat();
}

void ESAT_ADCSClass::handleWheelControllerSetIntegralGainCommand(ESAT_CCSDSPacket& packet)
{
  ESAT_WheelPIDController.integralGain = packet.readFloat();
}

void ESAT_ADCSClass::handleWheelControllerSetDerivativeGainCommand(ESAT_CCSDSPacket& packet)
{
  ESAT_WheelPIDController.derivativeGain = packet.readFloat();
}

void ESAT_ADCSClass::handleWheelControllerResetSpeedErrorIntegral(ESAT_CCSDSPacket& packet)
{
  ESAT_WheelPIDController.resetErrorIntegral();
}

void ESAT_ADCSClass::handleMagnetorquerEnableCommand(ESAT_CCSDSPacket& packet)
{
  runCode = MAGNETORQUER_ENABLE;
  ESAT_MagnetorquerEnableRunMode.enable = packet.readBoolean();
}

void ESAT_ADCSClass::handleMagnetorquerSetXPolarityCommand(ESAT_CCSDSPacket& packet)
{
  runCode = MAGNETORQUER_SET_X_POLARITY;
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    ESAT_MagnetorquerSetXPolarityRunMode.polarity = ESAT_Magnetorquer.POSITIVE;
  }
  else
  {
    ESAT_MagnetorquerSetXPolarityRunMode.polarity = ESAT_Magnetorquer.NEGATIVE;
  }
}

void ESAT_ADCSClass::handleMagnetorquerSetYPolarityCommand(ESAT_CCSDSPacket& packet)
{
  runCode = MAGNETORQUER_SET_Y_POLARITY;
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    ESAT_MagnetorquerSetYPolarityRunMode.polarity = ESAT_Magnetorquer.POSITIVE;
  }
  else
  {
    ESAT_MagnetorquerSetYPolarityRunMode.polarity = ESAT_Magnetorquer.NEGATIVE;
  }
}

void ESAT_ADCSClass::handleMagnetorquerApplyMaximumTorqueCommand(ESAT_CCSDSPacket& packet)
{
  runCode = MAGNETORQUER_APPLY_MAXIMUM_TORQUE;
  const byte parameter = packet.readByte();
  if (parameter == 0)
  {
    ESAT_MagnetorquerApplyMaximumTorqueRunMode.mode =
      ESAT_MagnetorquerApplyMaximumTorqueRunMode.ROTATE_CLOCKWISE;
  }
  else
  {
    ESAT_MagnetorquerApplyMaximumTorqueRunMode.mode =
      ESAT_MagnetorquerApplyMaximumTorqueRunMode.ROTATE_COUNTERCLOCKWISE;
  }
}

void ESAT_ADCSClass::handleMagnetorquerDemagnetizeCommand(ESAT_CCSDSPacket& packet)
{
  runCode = MAGNETORQUER_DEMAGNETIZE;
  ESAT_MagnetorquerDemagnetizeRunMode.cycles = packet.readByte();
}

void ESAT_ADCSClass::handleStopActuatorsCommand(ESAT_CCSDSPacket& packet)
{
  runCode = STOP_ACTUATORS;
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
  packet.writeByte(byte(runCode));
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

void ESAT_ADCSClass::run()
{
  switch (runCode)
  {
  case FOLLOW_MAGNETIC_ANGLE:
    runFollowMagneticAngle();
    break;
  case FOLLOW_SUN_ANGLE:
    runFollowSunAngle();
    break;
  case WHEEL_SET_DUTY_CYCLE:
    runWheelSetDutyCycle();
    break;
  case WHEEL_SET_SPEED:
    runWheelSetSpeed();
    break;
  case MAGNETORQUER_ENABLE:
    runMagnetorquerEnable();
    break;
  case MAGNETORQUER_APPLY_MAXIMUM_TORQUE:
    runMagnetorquerApplyMaximumTorque();
    break;
  case MAGNETORQUER_DEMAGNETIZE:
    runMagnetorquerDemagnetize();
    break;
  default:
    runStopActuators();
    break;
  }
}

void ESAT_ADCSClass::runFollowMagneticAngle()
{
  ESAT_FollowMagneticAngleRunMode.loop(attitudeStateVector);
}

void ESAT_ADCSClass::runFollowSunAngle()
{
  ESAT_FollowSunAngleRunMode.loop(attitudeStateVector);
}

void ESAT_ADCSClass::runWheelSetDutyCycle()
{
  ESAT_WheelSetDutyCycleRunMode.loop(attitudeStateVector);
}

void ESAT_ADCSClass::runWheelSetSpeed()
{
  ESAT_WheelSetSpeedRunMode.loop(attitudeStateVector);
}

void ESAT_ADCSClass::runMagnetorquerEnable()
{
  ESAT_MagnetorquerEnableRunMode.loop(attitudeStateVector);
}

void ESAT_ADCSClass::runMagnetorquerSetXPolarity()
{
  ESAT_MagnetorquerSetXPolarityRunMode.loop(attitudeStateVector);
}

void ESAT_ADCSClass::runMagnetorquerSetYPolarity()
{
  ESAT_MagnetorquerSetYPolarityRunMode.loop(attitudeStateVector);
}

void ESAT_ADCSClass::runMagnetorquerApplyMaximumTorque()
{
  ESAT_MagnetorquerApplyMaximumTorqueRunMode.loop(attitudeStateVector);
}

void ESAT_ADCSClass::runMagnetorquerDemagnetize()
{
  ESAT_MagnetorquerDemagnetizeRunMode.loop(attitudeStateVector);
}

void ESAT_ADCSClass::runStopActuators()
{
  ESAT_StopActuatorsRunMode.loop(attitudeStateVector);
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
