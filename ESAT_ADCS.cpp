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
#include "ESAT_CoarseSunSensor.h"
#include "ESAT_Gyroscope.h"
#include "ESAT_Magnetometer.h"
#include "ESAT_Magnetorquer.h"
#include "ESAT_Tachometer.h"
#include "ESAT_Wheel.h"
#include "ESAT_Timestamp.h"
#include "ESAT_OBCClock.h"

void ESAT_ADCSClass::begin(const word periodMilliseconds)
{
  actuator = MAGNETORQUER;
  attitudeDerivativeGain = 4e4;
  attitudeErrorDeadband = 1;
  attitudeErrorDerivativeDeadband = 2;
  attitudeErrorDerivativeDetumblingThreshold = 40;
  attitudeErrorIntegral = 0;
  attitudeIntegralGain = 0.0;
  attitudeProportionalGain = 1e3;
  demagnetizationIterations = 0;
  enableMagnetorquerDriver = false;
  magnetorquerXPolarity = ESAT_Magnetorquer.POSITIVE;
  magnetorquerYPolarity = ESAT_Magnetorquer.POSITIVE;
  newTelemetryPacket = false;
  oldAttitudeError = 0;
  oldWheelSpeedError = 0;
  period = periodMilliseconds;
  runCode = REST;
  rotationalSpeed = 0;
  targetAttitude = 0;
  targetMagnetorquerDirection = false;
  targetWheelSpeed = 0;
  telemetryPacketSequenceCount = 0;
  useGyroscope = true;
  wheelDerivativeGain = 0;
  wheelIntegralGain = 8.5e-4;
  wheelProportionalGain = 8.5e-3;
  wheelSpeedErrorIntegral = 0;
  wheelDutyCycle = 0;
  wheelSpeed = 0;
  ESAT_Wheel.begin();
  ESAT_Gyroscope.begin(ESAT_Gyroscope.FULL_SCALE_2000_DEGREES_PER_SECOND);
  ESAT_Magnetometer.begin();
  ESAT_CoarseSunSensor.begin();
  ESAT_Tachometer.begin();
  ESAT_Magnetorquer.begin();
  blinkSequence();
}

void ESAT_ADCSClass::blinkSequence()
{
  const uint32_t sequence = 1147235945;
  for (int i = 0; i < 32; i++)
  {
    const int state = ((sequence>>i) & 1);
    if (state == 1)
    {
      ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.POSITIVE);
      ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.POSITIVE);
    }
    else
    {
      ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.NEGATIVE);
      ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.NEGATIVE);
    }
    delay(50);
  }
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
    case FOLLOW_MAGNETOMETER_COMMAND:
      handleFollowMagnetometerCommand(packet);
      break;
    case FOLLOW_SUN_COMMAND:
      handleFollowSunCommand(packet);
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
    case REST_COMMAND:
      handleRestCommand(packet);
    default:
      break;
  }
}

void ESAT_ADCSClass::handleFollowMagnetometerCommand(ESAT_CCSDSPacket& packet)
{
  runCode = FOLLOW_MAGNETOMETER;
  const word rawTargetAttitude = packet.readWord();
  targetAttitude = rawTargetAttitude % 360;
  attitudeErrorIntegral = 0;
  oldAttitudeError = 0;
}

void ESAT_ADCSClass::handleFollowSunCommand(ESAT_CCSDSPacket& packet)
{
  runCode = FOLLOW_SUN;
  const word rawTargetAttitude = packet.readWord();
  targetAttitude = rawTargetAttitude % 360;
  attitudeErrorIntegral = 0;
  oldAttitudeError = 0;
}

void ESAT_ADCSClass::handleAttitudeControllerSetProportionalGainCommand(ESAT_CCSDSPacket& packet)
{
  attitudeProportionalGain = packet.readFloat();
  attitudeErrorIntegral = 0;
  oldAttitudeError = 0;
}

void ESAT_ADCSClass::handleAttitudeControllerSetIntegralGainCommand(ESAT_CCSDSPacket& packet)
{
  attitudeIntegralGain = packet.readFloat();
  attitudeErrorIntegral = 0;
  oldAttitudeError = 0;
}

void ESAT_ADCSClass::handleAttitudeControllerSetDerivativeGainCommand(ESAT_CCSDSPacket& packet)
{
  attitudeDerivativeGain = packet.readFloat();
  attitudeErrorIntegral = 0;
  oldAttitudeError = 0;
}

void ESAT_ADCSClass::handleAttitudeControllerUseGyroscopeCommand(ESAT_CCSDSPacket& packet)
{
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    useGyroscope = true;
  }
  else
  {
    useGyroscope = false;
  }
}

void ESAT_ADCSClass::handleAttitudeControllerSetActuatorCommand(ESAT_CCSDSPacket& packet)
{
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    actuator = WHEEL;
  }
  else
  {
    actuator = MAGNETORQUER;
  }
}

void ESAT_ADCSClass::handleAttitudeControllerSetDeadbandCommand(ESAT_CCSDSPacket& packet)
{
  attitudeErrorDeadband = packet.readWord();
  attitudeErrorDerivativeDeadband = packet.readWord();
}

void ESAT_ADCSClass::handleAttitudeControllerSetDetumblingThresholdCommand(ESAT_CCSDSPacket& packet)

{
  attitudeErrorDerivativeDetumblingThreshold = packet.readWord();
}

void ESAT_ADCSClass::handleWheelSetDutyCycleCommand(ESAT_CCSDSPacket& packet)
{
  runCode = WHEEL_SET_DUTY_CYCLE;
  wheelDutyCycle = packet.readFloat();
}

void ESAT_ADCSClass::handleWheelSetSpeedCommand(ESAT_CCSDSPacket& packet)
{
  runCode = WHEEL_SET_SPEED;
  word rawTargetWheelSpeed = packet.readWord();
  targetWheelSpeed = constrain(rawTargetWheelSpeed, 0, 8000);
  if (targetWheelSpeed == 0)
  {
    runCode = WHEEL_SET_DUTY_CYCLE;
    ESAT_Wheel.writeDutyCycle(128);
  }
}

void ESAT_ADCSClass::handleWheelControllerSetProportionalGainCommand(ESAT_CCSDSPacket& packet)
{
  wheelProportionalGain = packet.readFloat();
  wheelSpeedErrorIntegral = 0;
  oldWheelSpeedError = 0;
}

void ESAT_ADCSClass::handleWheelControllerSetIntegralGainCommand(ESAT_CCSDSPacket& packet)
{
  wheelIntegralGain = packet.readFloat();
  wheelSpeedErrorIntegral = 0;
  oldWheelSpeedError = 0;
}

void ESAT_ADCSClass::handleWheelControllerSetDerivativeGainCommand(ESAT_CCSDSPacket& packet)
{
  wheelDerivativeGain = packet.readFloat();
  wheelSpeedErrorIntegral = 0;
  oldWheelSpeedError = 0;
}

void ESAT_ADCSClass::handleMagnetorquerEnableCommand(ESAT_CCSDSPacket& packet)
{
  runCode = MAGNETORQUER_ENABLE;
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    enableMagnetorquerDriver = true;
  }
  else
  {
    enableMagnetorquerDriver = false;
  }
}

void ESAT_ADCSClass::handleMagnetorquerSetXPolarityCommand(ESAT_CCSDSPacket& packet)
{
  runCode = MAGNETORQUER_SET_X_POLARITY;
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    magnetorquerXPolarity = ESAT_Magnetorquer.POSITIVE;
  }
  else
  {
    magnetorquerXPolarity = ESAT_Magnetorquer.NEGATIVE;
  }
}

void ESAT_ADCSClass::handleMagnetorquerSetYPolarityCommand(ESAT_CCSDSPacket& packet)
{
  runCode = MAGNETORQUER_SET_Y_POLARITY;
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    magnetorquerYPolarity = ESAT_Magnetorquer.POSITIVE;
  }
  else
  {
    magnetorquerYPolarity = ESAT_Magnetorquer.NEGATIVE;
  }
}

void ESAT_ADCSClass::handleMagnetorquerApplyMaximumTorqueCommand(ESAT_CCSDSPacket& packet)
{
  runCode = MAGNETORQUER_APPLY_MAXIMUM_TORQUE;
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    targetMagnetorquerDirection = true;
  }
  else
  {
    targetMagnetorquerDirection = false;
  }
}

void ESAT_ADCSClass::handleMagnetorquerDemagnetizeCommand(ESAT_CCSDSPacket& packet)
{
  runCode = MAGNETORQUER_DEMAGNETIZE;
  demagnetizationIterations = packet.readByte();
}

void ESAT_ADCSClass::handleRestCommand(ESAT_CCSDSPacket& packet)
{
  runCode = REST;
}

void ESAT_ADCSClass::readSensors()
{
  wheelSpeed = ESAT_Tachometer.read();
  ESAT_Magnetorquer.writeEnable(false);
  delay(20);
  ESAT_Gyroscope.error = false;
  rotationalSpeed = ESAT_Gyroscope.read(3);
  ESAT_Magnetometer.error = false;
  magneticAngle = ESAT_Magnetometer.read();
  ESAT_Magnetorquer.writeEnable(enableMagnetorquerDriver);
  sunAngle = ESAT_CoarseSunSensor.read();
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
  packet.writeWord(targetAttitude);
  packet.writeWord(magneticAngle);
  packet.writeWord(sunAngle);
  packet.writeWord(rotationalSpeed);
  packet.writeFloat(attitudeProportionalGain);
  packet.writeFloat(attitudeIntegralGain);
  packet.writeFloat(attitudeDerivativeGain);
  packet.writeBoolean(useGyroscope);
  packet.writeByte(actuator);
  packet.writeFloat(wheelDutyCycle);
  packet.writeWord(wheelSpeed);
  packet.writeFloat(wheelProportionalGain);
  packet.writeFloat(wheelIntegralGain);
  packet.writeFloat(wheelDerivativeGain);
  packet.writeByte(enableMagnetorquerDriver);
  packet.writeByte(byte(magnetorquerXPolarity));
  packet.writeByte(byte(magnetorquerYPolarity));
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
  case FOLLOW_MAGNETOMETER:
    runFollowMagnetometer();
    break;
  case FOLLOW_SUN:
    runFollowSun();
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
    runRest();
    break;
  }
}

void ESAT_ADCSClass::runAttitudeControlLoop(int currentAttitude)
{
  int attitudeError = targetAttitude - currentAttitude;
  if (attitudeError > 180)
  {
    attitudeError = attitudeError - 360;
  }
  else if (attitudeError < -180)
  {
    attitudeError = attitudeError + 360;
  }
  int attitudeErrorDerivative;
  if (useGyroscope)
  {
    attitudeErrorDerivative = rotationalSpeed;
  }
  else
  {
    attitudeErrorDerivative =
      (attitudeError - oldAttitudeError) * (1000. / period);
  }
  float Kp = attitudeProportionalGain;
  float Kd = attitudeDerivativeGain;
  float Ki = attitudeIntegralGain;
  if ((abs(attitudeError) < attitudeErrorDeadband)
      && (abs(attitudeErrorDerivative) < attitudeErrorDerivativeDeadband))
  {
    Ki = 0;
    Kp = 0;
    Kd = 0;
  }
  if (abs(attitudeErrorDerivative) > attitudeErrorDerivativeDetumblingThreshold)
  {
    Ki = 0;
    Kp = 0;
  }
  float actuation = Kp * attitudeError
                  + Kd * rotationalSpeed
                  + Ki * attitudeErrorIntegral;
  attitudeErrorIntegral =
    attitudeErrorIntegral
    + attitudeError * (period / 1000.);
  if (actuator == WHEEL)
  {
    targetWheelSpeed = constrain(wheelSpeed + actuation, 0, 8000);
    runWheelSetSpeed();
  }
  else
  {
    if (actuation > 0)
    {
      targetMagnetorquerDirection = false;
    }
    else
    {
      targetMagnetorquerDirection = true;
    }
    runMagnetorquerApplyMaximumTorque();
  }
}

void ESAT_ADCSClass::runFollowMagnetometer()
{
  runAttitudeControlLoop(magneticAngle);
}

void ESAT_ADCSClass::runFollowSun()
{
  runAttitudeControlLoop(sunAngle);
}

void ESAT_ADCSClass::runWheelSetDutyCycle()
{
  ESAT_Wheel.writeDutyCycle(wheelDutyCycle);
}

void ESAT_ADCSClass::runWheelSetSpeed()
{
  const int wheelSpeedError = targetWheelSpeed - wheelSpeed;
  wheelSpeedErrorIntegral = wheelSpeedErrorIntegral + wheelSpeedError;
  const int wheelSpeedErrorDerivative = wheelSpeedError - oldWheelSpeedError;
  oldWheelSpeedError = wheelSpeedError;
  const float control = wheelProportionalGain * wheelSpeedError
    + wheelIntegralGain * wheelSpeedErrorIntegral
    + wheelDerivativeGain * wheelSpeedErrorDerivative;
  ESAT_Wheel.write(control);
}

void ESAT_ADCSClass::runMagnetorquerEnable()
{
  if (magnetorquerXPolarity > 0)
  {
    ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.POSITIVE);
  }
  else
  {
    ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.NEGATIVE);
  }
  if (magnetorquerYPolarity > 0)
  {
    ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.POSITIVE);
  }
  else
  {
    ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.NEGATIVE);
  }
}

void ESAT_ADCSClass::runMagnetorquerSetXPolarity()
{
  if (magnetorquerXPolarity > 0)
  {
    ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.POSITIVE);
  }
  else
  {
    ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.NEGATIVE);
  }
}

void ESAT_ADCSClass::runMagnetorquerSetYPolarity()
{
  if (magnetorquerYPolarity > 0)
  {
    ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.POSITIVE);
  }
  else
  {
    ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.NEGATIVE);
  }
}

void ESAT_ADCSClass::runMagnetorquerApplyMaximumTorque()
{
  const bool activationsX[4] = {
          targetMagnetorquerDirection,
          targetMagnetorquerDirection,
          !targetMagnetorquerDirection,
          !targetMagnetorquerDirection
  };
  const bool activationsY[4] = {
          !targetMagnetorquerDirection,
          targetMagnetorquerDirection,
          targetMagnetorquerDirection,
          !targetMagnetorquerDirection
  };
  const long quadrant = map(magneticAngle % 360, 0, 360, 0, 4);
  if (activationsX[quadrant])
  {
    magnetorquerXPolarity = ESAT_Magnetorquer.POSITIVE;
    ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.POSITIVE);
  }
  else
  {
    magnetorquerXPolarity = ESAT_Magnetorquer.NEGATIVE;
    ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.NEGATIVE);
  }
  if (activationsY[quadrant])
  {
    magnetorquerYPolarity = ESAT_Magnetorquer.POSITIVE;
    ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.POSITIVE);
  }
  else
  {
    magnetorquerYPolarity = ESAT_Magnetorquer.NEGATIVE;
    ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.NEGATIVE);
  }
  enableMagnetorquerDriver = true;
  ESAT_Magnetorquer.writeEnable(enableMagnetorquerDriver);
}

void ESAT_ADCSClass::runMagnetorquerDemagnetize()
{
  ESAT_Magnetorquer.writeEnable(true);
  for (int i = 0; i < demagnetizationIterations; i++)
  {
    ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.POSITIVE);
    ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.POSITIVE);
    delay(20);
    ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.NEGATIVE);
    ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.NEGATIVE);
    delay(20);
  }
  ESAT_Magnetorquer.writeEnable(false);
  enableMagnetorquerDriver = false;
  runCode = REST;
}

void ESAT_ADCSClass::runRest()
{
  enableMagnetorquerDriver = false;
  ESAT_Magnetorquer.writeEnable(false);
  wheelDutyCycle = 0;
  ESAT_Wheel.writeDutyCycle(wheelDutyCycle);
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
