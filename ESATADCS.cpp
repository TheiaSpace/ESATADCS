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

void ESATADCS::begin()
{
  attitudeDerivativeGain = 4e4;
  attitudeErrorIntegral = 0;
  attitudeIntegralGain = 0.0;
  attitudeProportionalGain = 1e3;
  demagnetizationIterations = 0;
  enableMagnetorquerDriver = false;
  magnetorquerXPolarity = Magnetorquer.positive;
  magnetorquerYPolarity = Magnetorquer.positive;
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
    Magnetorquer.writeX(state);
    Magnetorquer.writeY(state);
    delay(50);
  }
}

void ESATADCS::handleCommand(ESATCCSDSPacket& packet)
{
  if (packet.bufferLength < MINIMUM_COMMAND_PACKET_LENGTH)
  {
    return;
  }
  packet.rewind();
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
    magnetorquerXPolarity = Magnetorquer.positive;
  }
  else
  {
    magnetorquerYPolarity = Magnetorquer.negative;
  }
}

void ESATADCS::handleMagnetorquerSetYPolarityCommand(ESATCCSDSPacket& packet)
{
  runCode = MAGNETORQUER_SET_Y_POLARITY;
  const byte parameter = packet.readByte();
  if (parameter > 0)
  {
    magnetorquerYPolarity = Magnetorquer.positive;
  }
  else
  {
    magnetorquerYPolarity = Magnetorquer.negative;
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
  rotationalSpeed = Gyroscope.read(3);
  magneticAngle = Magnetometer.read();
  Magnetorquer.writeEnable(enableMagnetorquerDriver);
  sunAngle = CoarseSunSensor.read();
}

void ESATADCS::readTelemetry(ESATCCSDSPacket& packet)
{
  newTelemetryPacket = false;
  packet.clear();
  if (packet.bufferLength < packet.PRIMARY_HEADER_LENGTH)
  {
    return;
  }
  packet.writePacketVersionNumber(0);
  packet.writePacketType(packet.TELEMETRY);
  packet.writeSecondaryHeaderFlag(packet.SECONDARY_HEADER_IS_PRESENT);
  packet.writeApplicationProcessIdentifier(SUBSYSTEM_IDENTIFIER);
  packet.writeSequenceFlags(packet.UNSEGMENTED_USER_DATA);
  packet.writePacketSequenceCount(telemetryPacketSequenceCount);
  packet.writeByte(MAJOR_VERSION_NUMBER);
  packet.writeByte(MINOR_VERSION_NUMBER);
  packet.writeByte(PATCH_VERSION_NUMBER);
  packet.writeByte(HOUSEKEEPING);
  packet.writeWord(wheelSpeed);
  packet.writeWord(magneticAngle);
  packet.writeWord(sunAngle);
  packet.writeWord(rotationalSpeed);
  packet.writeByte(enableMagnetorquerDriver);
  if (magnetorquerXPolarity == Magnetorquer.positive)
  {
    packet.writeByte(1);
  }
  else
  {
    packet.writeByte(0);
  }
  if (magnetorquerYPolarity == Magnetorquer.positive)
  {
    packet.writeByte(1);
  }
  else
  {
    packet.writeByte(0);
  }
  if (Gyroscope.alive)
  {
    packet.writeByte(1);
  }
  else
  {
    packet.writeByte(0);
  }
  if (Magnetometer.alive)
  {
    packet.writeByte(1);
  }
  else
  {
    packet.writeByte(0);
  }
  telemetryPacketSequenceCount = telemetryPacketSequenceCount + 1;
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
  const int absoluteAttitudeErrorDerivative = abs(attitudeErrorDerivative);
  //If the rotation is faster than 40º/s no control is feasible and
  //only rotation damping is considered.
  if (absoluteAttitudeErrorDerivative > 40)
  {
    Ki = 0;
    Kp = 0;
  }
  //If the rotation speed is smaller than 2º/seconds, forget the
  //derivative gain (TBC)
  if (absoluteAttitudeErrorDerivative <= 2)
  {
    Kd = 0;
  }
  //If the solution is found, stop the controller to avoid instabilities
  if (abs(attitudeError) < 1 && absoluteAttitudeErrorDerivative <= 2)
  {
    Ki = 0;
    Kp = 0;
    Kd = 0;
  }
  //PID control itself
  float actuation = Kp * attitudeError
                  + Kd * rotationalSpeed
                  + Ki * attitudeErrorIntegral;
  //If going against the wheel, increase actuation to magnify results
  if (actuation < 0)
  {
    actuation *= 3;
  }
  attitudeErrorIntegral = attitudeErrorIntegral + attitudeError;
  //Selection of preferred actuation
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
  Magnetorquer.writeX(magnetorquerXPolarity);
  Magnetorquer.writeY(magnetorquerYPolarity);
  Magnetorquer.writeEnable(enableMagnetorquerDriver);
}

void ESATADCS::runMagnetorquerSetXPolarity()
{
  Magnetorquer.writeX(magnetorquerXPolarity);
}

void ESATADCS::runMagnetorquerSetYPolarity()
{
  Magnetorquer.writeY(magnetorquerYPolarity);
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
  magnetorquerXPolarity =
    activationsX[quadrant] ? Magnetorquer.positive : Magnetorquer.negative;
  magnetorquerYPolarity =
    activationsY[quadrant] ? Magnetorquer.positive : Magnetorquer.negative;
  enableMagnetorquerDriver = true;
  Magnetorquer.writeX(magnetorquerXPolarity);
  Magnetorquer.writeY(magnetorquerYPolarity);
  Magnetorquer.writeEnable(enableMagnetorquerDriver);
}

void ESATADCS::runMagnetorquerDemagnetize()
{
  Magnetorquer.writeEnable(true);
  for (long  i = 0; i < demagnetizationIterations; i++)
  {
    Magnetorquer.writeX(Magnetorquer.positive);
    Magnetorquer.writeY(Magnetorquer.positive);
    delay(20);
    Magnetorquer.writeX(Magnetorquer.negative);
    Magnetorquer.writeY(Magnetorquer.negative);
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
