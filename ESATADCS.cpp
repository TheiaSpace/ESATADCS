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

#include "ESATADCS.h"
#include "ESATCoarseSunSensor.h"
#include "ESATGyroscope.h"
#include "ESATMagnetometer.h"
#include "ESATMagnetorquer.h"
#include "ESATTachometer.h"
#include <ESATUtil.h>
#include "ESATWheel.h"
#include "ESATTimestamp.h"
#include "ESATClock.h"

void ESATADCS::begin()
{
  attitudeDerivativeGain = 4e4;
  attitudeErrorDeadband = 1;
  attitudeErrorDerivativeDeadband = 2;
  attitudeErrorDerivativeDetumblingThreshold = 40;
  attitudeErrorIntegral = 0;
  attitudeIntegralGain = 0.0;
  attitudeProportionalGain = 1e3;
  demagnetizationIterations = 0;
  enableMagnetorquerDriver = false;
  magnetorquerXPolarity = Magnetorquer.POSITIVE;
  magnetorquerYPolarity = Magnetorquer.POSITIVE;
  newTelemetryPacket = false;
  oldAttitudeError = 0;
  oldWheelSpeedError = 0;
  runCode = REST;
  rotationalSpeed = 0;
  targetAttitude = 0;
  targetMagnetorquerDirection = false;
  targetWheelSpeed = 0;
  telemetryPacketSequenceCount = 0;
  useGyroscope = true;
  useWheel = true;
  wheelDerivativeGain = 0;
  wheelIntegralGain = 8.5e-4;
  wheelProportionalGain = 8.5e-3;
  wheelSpeedErrorIntegral = 0;
  wheelDutyCycle = 128;
  wheelSpeed = 0;
  Wheel.begin();
  Gyroscope.begin(Gyroscope.FULL_SCALE_2000_DEGREES_PER_SECOND);
  Magnetometer.begin();
  CoarseSunSensor.begin();
  Tachometer.begin();
  Magnetorquer.begin();
  blinkSequence();
}

void ESATADCS::blinkSequence()
{
  const uint32_t sequence = 1147235945;
  for (int i = 0; i < 32; i++)
  {
    const int state = ((sequence>>i) & 1);
    if (state == 1)
    {
      Magnetorquer.writeX(Magnetorquer.POSITIVE);
      Magnetorquer.writeY(Magnetorquer.POSITIVE);
    }
    else
    {
      Magnetorquer.writeX(Magnetorquer.NEGATIVE);
      Magnetorquer.writeY(Magnetorquer.NEGATIVE);
    }
    delay(50);
  }
}

word ESATADCS::getApplicationProcessIdentifier()
{
  return APPLICATION_PROCESS_IDENTIFIER;
}

void ESATADCS::handleTelecommand(ESATCCSDSPacket& packet)
{
  packet.rewind();
  if (packet.readApplicationProcessIdentifier()
      != APPLICATION_PROCESS_IDENTIFIER)
  {
    return;
  }
  if (packet.readPacketType() != packet.TELECOMMAND)
  {
    return;
  }
  if (packet.readPacketDataLength() < MINIMUM_COMMAND_PAYLOAD_DATA_LENGTH)
  {
    return;
  }
  ESATTimestamp Timestamp;
  Timestamp.year = packet.readWord() - 2000;
  Timestamp.month = packet.readByte();
  Timestamp.day = packet.readByte();
  Timestamp.hours = packet.readByte();
  Timestamp.minutes = packet.readByte();
  Timestamp.seconds = packet.readByte();
  // Nothing to do with this timestamp until the scheduled
  // commands are implemented
  const byte majorVersionNumber = packet.readByte();
  const byte minorVersionNumber = packet.readByte();
  const byte patchVersionNumber = packet.readByte();
  if (majorVersionNumber < MAJOR_VERSION_NUMBER)
  {
    return;
  }
  const byte commandCode = packet.readByte();
  switch (commandCode)
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
  case ATTITUDE_CONTROLLER_USE_WHEEL_OR_MAGNETORQUER_COMMAND:
    handleAttitudeControllerUseWheelOrMagnetorquerCommand(packet);
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

void ESATADCS::handleFollowMagnetometerCommand(ESATCCSDSPacket& packet)
{
  runCode = FOLLOW_MAGNETOMETER;
  const word rawTargetAttitude = packet.readWord();
  targetAttitude = rawTargetAttitude % 360;
  attitudeErrorIntegral = 0;
  oldAttitudeError = 0;
}

void ESATADCS::handleFollowSunCommand(ESATCCSDSPacket& packet)
{
  runCode = FOLLOW_SUN;
  const word rawTargetAttitude = packet.readWord();
  targetAttitude = rawTargetAttitude % 360;
  attitudeErrorIntegral = 0;
  oldAttitudeError = 0;
}

void ESATADCS::handleAttitudeControllerSetProportionalGainCommand(ESATCCSDSPacket& packet)
{
  attitudeProportionalGain = packet.readFloat();
  attitudeErrorIntegral = 0;
  oldAttitudeError = 0;
}

void ESATADCS::handleAttitudeControllerSetIntegralGainCommand(ESATCCSDSPacket& packet)
{
  attitudeIntegralGain = packet.readFloat();
  attitudeErrorIntegral = 0;
  oldAttitudeError = 0;
}

void ESATADCS::handleAttitudeControllerSetDerivativeGainCommand(ESATCCSDSPacket& packet)
{
  attitudeIntegralGain = packet.readFloat();
  attitudeErrorIntegral = 0;
  oldAttitudeError = 0;
}

void ESATADCS::handleAttitudeControllerUseGyroscopeCommand(ESATCCSDSPacket& packet)
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

void ESATADCS::handleAttitudeControllerUseWheelOrMagnetorquerCommand(ESATCCSDSPacket& packet)
{
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    useWheel = true;
  }
  else
  {
    useWheel = false;
  }
}

void ESATADCS::handleAttitudeControllerSetDeadbandCommand(ESATCCSDSPacket& packet)
{
  attitudeErrorDeadband = packet.readWord();
  attitudeErrorDerivativeDeadband = packet.readWord();
}

void ESATADCS::handleAttitudeControllerSetDetumblingThresholdCommand(ESATCCSDSPacket& packet)

{
  attitudeErrorDerivativeDetumblingThreshold = packet.readWord();
}

void ESATADCS::handleWheelSetDutyCycleCommand(ESATCCSDSPacket& packet)
{
  runCode = WHEEL_SET_DUTY_CYCLE;
  wheelDutyCycle = packet.readByte();
}

void ESATADCS::handleWheelSetSpeedCommand(ESATCCSDSPacket& packet)
{
  runCode = WHEEL_SET_SPEED;
  word rawTargetWheelSpeed = packet.readWord();
  targetWheelSpeed = constrain(rawTargetWheelSpeed, 0, 8000);
  if (targetWheelSpeed == 0)
  {
    runCode = WHEEL_SET_DUTY_CYCLE;
    Wheel.writeDutyCycle(128);
  }
}

void ESATADCS::handleWheelControllerSetProportionalGainCommand(ESATCCSDSPacket& packet)
{
  wheelProportionalGain = packet.readFloat();
  wheelSpeedErrorIntegral = 0;
  oldWheelSpeedError = 0;
}

void ESATADCS::handleWheelControllerSetIntegralGainCommand(ESATCCSDSPacket& packet)
{
  wheelIntegralGain = packet.readFloat();
  wheelSpeedErrorIntegral = 0;
  oldWheelSpeedError = 0;
}

void ESATADCS::handleWheelControllerSetDerivativeGainCommand(ESATCCSDSPacket& packet)
{
  wheelDerivativeGain = packet.readFloat();
  wheelSpeedErrorIntegral = 0;
  oldWheelSpeedError = 0;
}

void ESATADCS::handleMagnetorquerEnableCommand(ESATCCSDSPacket& packet)
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

void ESATADCS::handleMagnetorquerSetXPolarityCommand(ESATCCSDSPacket& packet)
{
  runCode = MAGNETORQUER_SET_X_POLARITY;
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    magnetorquerXPolarity = Magnetorquer.POSITIVE;
  }
  else
  {
    magnetorquerYPolarity = Magnetorquer.NEGATIVE;
  }
}

void ESATADCS::handleMagnetorquerSetYPolarityCommand(ESATCCSDSPacket& packet)
{
  runCode = MAGNETORQUER_SET_Y_POLARITY;
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    magnetorquerYPolarity = Magnetorquer.POSITIVE;
  }
  else
  {
    magnetorquerYPolarity = Magnetorquer.NEGATIVE;
  }
}

void ESATADCS::handleMagnetorquerApplyMaximumTorqueCommand(ESATCCSDSPacket& packet)
{
  runCode = MAGNETORQUER_APPLY_MAXIMUM_TORQUE;
  const byte parameter = packet.readWord();
  if (parameter > 0)
  {
    targetMagnetorquerDirection = true;
  }
  else
  {
    targetMagnetorquerDirection = false;
  }
}

void ESATADCS::handleMagnetorquerDemagnetizeCommand(ESATCCSDSPacket& packet)
{
  runCode = MAGNETORQUER_DEMAGNETIZE;
  demagnetizationIterations = packet.readWord();
}

void ESATADCS::handleRestCommand(ESATCCSDSPacket& packet)
{
  runCode = REST;
}

void ESATADCS::readSensors()
{
  wheelSpeed = Tachometer.read();
  Magnetorquer.writeEnable(false);
  delay(20);
  Gyroscope.error = false;
  rotationalSpeed = Gyroscope.read(3);
  Magnetometer.error = false;
  magneticAngle = Magnetometer.read();
  Magnetorquer.writeEnable(enableMagnetorquerDriver);
  sunAngle = CoarseSunSensor.read();
}

boolean ESATADCS::readTelemetry(ESATCCSDSPacket& packet)
{
  if (!newTelemetryPacket)
  {
    return false;
  }
  newTelemetryPacket = false;
  packet.clear();
  if (packet.packetDataBufferLength < HOUSEKEEPING_TELEMETRY_PACKET_LENGTH)
  {
    return false;
  }
  ESATTimestamp Timestamp = Clock.read();
  if(Clock.error)
  {
    return false;
  }
  // Primary header
  packet.writePacketVersionNumber(0);
  packet.writePacketType(packet.TELEMETRY);
  packet.writeSecondaryHeaderFlag(packet.SECONDARY_HEADER_IS_PRESENT);
  packet.writeApplicationProcessIdentifier(getApplicationProcessIdentifier());
  packet.writeSequenceFlags(packet.UNSEGMENTED_USER_DATA);
  packet.writePacketSequenceCount(telemetryPacketSequenceCount);
  // Secondary header
  packet.writeWord((word)Timestamp.year + 2000);
  packet.writeByte(Timestamp.month);
  packet.writeByte(Timestamp.day);
  packet.writeByte(Timestamp.hours);
  packet.writeByte(Timestamp.minutes);
  packet.writeByte(Timestamp.seconds);
  packet.writeByte(MAJOR_VERSION_NUMBER);
  packet.writeByte(MINOR_VERSION_NUMBER);
  packet.writeByte(PATCH_VERSION_NUMBER);
  packet.writeByte(HOUSEKEEPING);
  // User data
  packet.writeByte(byte(runCode));
  packet.writeWord(targetAttitude);
  packet.writeWord(magneticAngle);
  packet.writeWord(sunAngle);
  packet.writeWord(rotationalSpeed);
  packet.writeFloat(attitudeProportionalGain);
  packet.writeFloat(attitudeIntegralGain);
  packet.writeFloat(attitudeDerivativeGain);
  packet.writeBoolean(useGyroscope);
  packet.writeBoolean(useWheel);
  packet.writeByte(wheelDutyCycle);
  packet.writeWord(wheelSpeed);
  packet.writeFloat(wheelProportionalGain);
  packet.writeFloat(wheelIntegralGain);
  packet.writeFloat(wheelDerivativeGain);
  packet.writeByte(enableMagnetorquerDriver);
  packet.writeByte(byte(magnetorquerXPolarity));
  packet.writeByte(byte(magnetorquerYPolarity));
  packet.writeBoolean(Gyroscope.error);
  packet.writeBoolean(Magnetometer.error);
  // end of user data
  packet.updatePacketDataLength();
  if (packet.readPacketDataLength() > packet.packetDataBufferLength)
  {
    return false;
  }
  telemetryPacketSequenceCount = telemetryPacketSequenceCount + 1;
  return true;
}

void ESATADCS::run()
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

void ESATADCS::runAttitudeControlLoop(int currentAttitude)
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
    attitudeErrorDerivative = attitudeError - oldAttitudeError;
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
  attitudeErrorIntegral = attitudeErrorIntegral + attitudeError;
  if (useWheel)
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

void ESATADCS::runFollowMagnetometer()
{
  runAttitudeControlLoop(magneticAngle);
}

void ESATADCS::runFollowSun()
{
  runAttitudeControlLoop(sunAngle);
}

void ESATADCS::runWheelSetDutyCycle()
{
  Wheel.writeDutyCycle(wheelDutyCycle);
}

void ESATADCS::runWheelSetSpeed()
{
  const int wheelSpeedError = targetWheelSpeed - wheelSpeed;
  wheelSpeedErrorIntegral = wheelSpeedErrorIntegral + wheelSpeedError;
  const int wheelSpeedErrorDerivative = wheelSpeedError - oldWheelSpeedError;
  oldWheelSpeedError = wheelSpeedError;
  const float control = wheelProportionalGain * wheelSpeedError
    + wheelIntegralGain * wheelSpeedErrorIntegral
    + wheelDerivativeGain * wheelSpeedErrorDerivative;
  Wheel.write(control);
}

void ESATADCS::runMagnetorquerEnable()
{
  if (magnetorquerXPolarity > 0)
  {
    Magnetorquer.writeX(Magnetorquer.POSITIVE);
  }
  else
  {
    Magnetorquer.writeX(Magnetorquer.NEGATIVE);
  }
  if (magnetorquerYPolarity > 0)
  {
    Magnetorquer.writeY(Magnetorquer.POSITIVE);
  }
  else
  {
    Magnetorquer.writeY(Magnetorquer.NEGATIVE);
  }
}

void ESATADCS::runMagnetorquerSetXPolarity()
{
  if (magnetorquerXPolarity > 0)
  {
    Magnetorquer.writeX(Magnetorquer.POSITIVE);
  }
  else
  {
    Magnetorquer.writeX(Magnetorquer.NEGATIVE);
  }
}

void ESATADCS::runMagnetorquerSetYPolarity()
{
  if (magnetorquerYPolarity > 0)
  {
    Magnetorquer.writeY(Magnetorquer.POSITIVE);
  }
  else
  {
    Magnetorquer.writeY(Magnetorquer.NEGATIVE);
  }
}

void ESATADCS::runMagnetorquerApplyMaximumTorque()
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
    magnetorquerXPolarity = Magnetorquer.POSITIVE;
    Magnetorquer.writeX(Magnetorquer.POSITIVE);
  }
  else
  {
    magnetorquerXPolarity = Magnetorquer.NEGATIVE;
    Magnetorquer.writeX(Magnetorquer.NEGATIVE);
  }
  if (activationsY[quadrant])
  {
    magnetorquerYPolarity = Magnetorquer.POSITIVE;
    Magnetorquer.writeY(Magnetorquer.POSITIVE);
  }
  else
  {
    magnetorquerYPolarity = Magnetorquer.NEGATIVE;
    Magnetorquer.writeY(Magnetorquer.NEGATIVE);
  }
  enableMagnetorquerDriver = true;
  Magnetorquer.writeEnable(enableMagnetorquerDriver);
}

void ESATADCS::runMagnetorquerDemagnetize()
{
  Magnetorquer.writeEnable(true);
  for (long  i = 0; i < demagnetizationIterations; i++)
  {
    Magnetorquer.writeX(Magnetorquer.POSITIVE);
    Magnetorquer.writeY(Magnetorquer.POSITIVE);
    delay(20);
    Magnetorquer.writeX(Magnetorquer.NEGATIVE);
    Magnetorquer.writeY(Magnetorquer.NEGATIVE);
    delay(20);
  }
  Magnetorquer.writeEnable(false);
  enableMagnetorquerDriver = false;
  runCode = REST;
}

void ESATADCS::runRest()
{
  enableMagnetorquerDriver = false;
  Magnetorquer.writeEnable(false);
  wheelDutyCycle = 128;
  Wheel.writeDutyCycle(wheelDutyCycle);
}

boolean ESATADCS::telemetryAvailable()
{
  return newTelemetryPacket;
}

void ESATADCS::update()
{
  readSensors();
  run();
  newTelemetryPacket = true;
}

ESATADCS ADCS;
