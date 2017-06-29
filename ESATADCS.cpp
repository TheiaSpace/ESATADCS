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
#include <ESATMath.h>
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
  oldWheelSpeedError = 0;
  runCode = REST;
  rotationalSpeed = 0;
  targetAttitude = 0;
  targetMagnetorquerDirection = false;
  targetWheelSpeed = 0;
  useWheel = true;
  wheelDerivativeGain = 0e-1;
  wheelIntegralGain = 30e-2;
  wheelProportionalGain = 15e-1;
  wheelSpeedErrorIntegral = 0;
  Wheel.begin();
  Gyroscope.begin();
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

void ESATADCS::handleCommand(int commandCode, String parameters)
{
  delay(5);
  switch (commandCode)
  {
    case MOTOR_DUTY_COMMAND:
      handleMotorDutyCommand(parameters);
      break;
    case PID_CONFIGURATION_COMMAND:
      handlePIDConfigurationCommand(parameters);
      break;
    case ENABLE_MAGNETORQUER_X_AND_MAGNETORQUER_Y_COMMAND:
      handleEnableMagnetorquerXAndMagnetorquerYCommand(parameters);
      break;
    case MAGNETORQUER_X_POLARITY_COMMAND:
      handleMagnetorquerXPolarityCommand(parameters);
      break;
    case MTQY_POLARITY_COMMAND:
      handleMTQYPolarityCommand(parameters);
      break;
    case FOLLOW_SUN_COMMAND:
      handleFollowSunCommand(parameters);
      break;
    case FOLLOW_MAG_COMMAND:
      handleFollowMagCommand(parameters);
      break;
    case SET_WHEEL_SPEED_COMMAND:
      handleSetWheelSpeedCommand(parameters);
      break;
    case MAX_MAG_TORQUE_COMMAND:
      handleMaxMagTorqueCommand(parameters);
      break;
    case WHEEL_PID_CONFIGURATION_COMMAND:
      handleWheelPIDConfigurationCommand(parameters);
      break;
    case MTQ_DEMAG_COMMAND:
      handleMTQDemagCommand(parameters);
      break;
    case WHEEL_OR_MTQ_COMMAND:
      handleWheelOrMTQCommand(parameters);
      break;
    case WHEEL_CALIBRATION1_COMMAND:
      handleWheelCalibration1Command(parameters);
      break;
    case WHEEL_CALIBRATION2_COMMAND:
      handleWheelCalibration2Command(parameters);
      break;
    case WHEEL_CALIBRATION3_COMMAND:
      handleWheelCalibration3Command(parameters);
      break;
    case FLASH_WRITE_COMMAND:
      handleFlashWriteCommand(parameters);
      break;
    case FLASH_WRITE_DEFAULTS_COMMAND:
      handleFlashWriteDefaultsCommand(parameters);
      break;
    default:
      break;
  }
}

void ESATADCS::handleMotorDutyCommand(String parameters)
{
  runCode = SET_MOTOR_DUTY;
  const int dutyCycle = constrain(parameters.toInt(), 0, 255);
  Wheel.writeDutyCycle(dutyCycle);
}

void ESATADCS::handlePIDConfigurationCommand(String parameters)
{
  attitudeProportionalGain = parameters.substring(0, 4).toInt();
  attitudeDerivativeGain = parameters.substring(4, 8).toInt() * 1e1;
  attitudeIntegralGain = parameters.substring(8, 12).toInt();
}

void ESATADCS::handleEnableMagnetorquerXAndMagnetorquerYCommand(String parameters)
{
  runCode = ENABLE_MAGNETORQUER_X_AND_MAGNETORQUER_Y;
  enableMagnetorquerDriver = !!(parameters.toInt());
  Magnetorquer.writeX(magnetorquerXPolarity);
  Magnetorquer.writeY(magnetorquerYPolarity);
  Magnetorquer.writeEnable(enableMagnetorquerDriver);
}

void ESATADCS::handleMagnetorquerXPolarityCommand(String parameters)
{
  magnetorquerXPolarity =
    (parameters.toInt() > 0) ? Magnetorquer.positive : Magnetorquer.negative;
  Magnetorquer.writeX(magnetorquerXPolarity);
}

void ESATADCS::handleMTQYPolarityCommand(String parameters)
{
  magnetorquerYPolarity =
    (parameters.toInt() > 0) ? Magnetorquer.positive : Magnetorquer.negative;
  Magnetorquer.writeY(magnetorquerYPolarity);
}

void ESATADCS::handleFollowSunCommand(String parameters)
{
  runCode = FOLLOW_SUN;
  const int rawTargetAttitude = parameters.toInt();
  targetAttitude = constrain(rawTargetAttitude, 0, 360);
}

void ESATADCS::handleFollowMagCommand(String parameters)
{
  runCode = FOLLOW_MAGNETOMETER;
  const int rawTargetAttitude = parameters.toInt();
  targetAttitude = constrain(rawTargetAttitude, 0, 360);
}

void ESATADCS::handleSetWheelSpeedCommand(String parameters)
{
  runCode = SET_WHEEL_SPEED;
  int rawTargetWheelSpeed = parameters.toInt();
  targetWheelSpeed = constrain(rawTargetWheelSpeed, -8000, 8000);
  if (targetWheelSpeed == 0)
  {
    runCode = SET_MOTOR_DUTY;
    Wheel.writeDutyCycle(128);
  }
}

void ESATADCS::handleMaxMagTorqueCommand(String parameters)
{
  runCode = MAXIMUM_MAGNETIC_TORQUE;
  targetMagnetorquerDirection = !!(parameters.toInt());
}

void ESATADCS::handleWheelPIDConfigurationCommand(String parameters)
{
  wheelProportionalGain = parameters.substring(0, 2).toInt() * 1e-1;
  wheelDerivativeGain = parameters.substring(2, 4).toInt() * 1e-1;
  wheelIntegralGain = parameters.substring(4, 6).toInt() * 1e-2;
  wheelSpeedErrorIntegral = 0;
  oldWheelSpeedError = 0;
}

void ESATADCS::handleMTQDemagCommand(String parameters)
{
  runCode = DEMAGNETIZE;
  demagnetizationIterations = parameters.toInt();
}

void ESATADCS::handleWheelOrMTQCommand(String parameters)
{
  useWheel = !!(parameters.toInt());
}

void ESATADCS::handleWheelCalibration1Command(String parameters)
{
  Wheel.calibration[0] = parameters.substring(0, 5).toInt() * 0.1;
}

void ESATADCS::handleWheelCalibration2Command(String parameters)
{
  Wheel.calibration[1] = 1e-5 * parameters.substring(0, 5).toInt();
}

void ESATADCS::handleWheelCalibration3Command(String parameters)
{
  Wheel.calibration[2] = 1e-9 * parameters.substring(6, 11).toInt();
}

void ESATADCS::handleFlashWriteCommand(String parameter)
{
  Wheel.saveCalibration();
}

void ESATADCS::handleFlashWriteDefaultsCommand(String parameter)
{
  Wheel.defaultCalibration();
  Wheel.saveCalibration();
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
  inertialMeasurementUnitAlive = Gyroscope.alive && Magnetometer.alive;
}

String ESATADCS::readTelemetry()
{
  String telemetry;
  telemetry += Util.intToHexadecimal(wheelSpeed);
  telemetry += Util.intToHexadecimal(magneticAngle);
  const int sunAngleRelativeToNorth =
          Math.modulo(sunAngle - magneticAngle, 360);
  telemetry += Util.intToHexadecimal(sunAngleRelativeToNorth);
  telemetry += Util.intToHexadecimal(rotationalSpeed);
  telemetry += Util.byteToHexadecimal(byte(0));
  const byte magnetorquerStatus =
    (enableMagnetorquerDriver ? 100 : 0)
    + ((magnetorquerXPolarity == Magnetorquer.positive) ? 10 : 0)
    + ((magnetorquerYPolarity == Magnetorquer.positive) ? 1 : 0);
  telemetry += Util.byteToHexadecimal(magnetorquerStatus);
  telemetry += Util.intToHexadecimal(sunAngle);
  return telemetry;
}

void ESATADCS::run()
{
  switch (runCode)
  {
    case FOLLOW_SUN:
      runFollowSun();
      break;
    case FOLLOW_MAGNETOMETER:
      runFollowMagnetometer();
      break;
    case MAXIMUM_MAGNETIC_TORQUE:
      runMaximumMagneticTorque();
      break;
    case DEMAGNETIZE:
      runDemagnetize();
      break;
    case SET_WHEEL_SPEED:
      runSetWheelSpeed();
      break;
    default:
      break;
  }
}

void ESATADCS::runAttitudeControlLoop(int currentAttitude)
{
  int angleError = targetAttitude - currentAttitude;
  if (angleError > 180)
  {
    angleError = angleError - 360;
  }
  else if (angleError < -180)
  {
    angleError = angleError + 360;
  }
  const float error = angleError / 360.;
  float Kp = attitudeProportionalGain;
  float Kd = attitudeDerivativeGain;
  float Ki = attitudeIntegralGain;
  const int absoluteRotationalSpeed = int(Math.abs(rotationalSpeed));
  //If the rotation is faster than 40º/s no control is feasible and
  //only rotation damping is considered.
  if (absoluteRotationalSpeed > 40)
  {
    Ki = 0;
    Kp = 0;
  }
  //If the rotation speed is smaller than 2º/seconds, forget the
  //derivative gain (TBC)
  if (absoluteRotationalSpeed <= 2)
  {
    Kd = 0;
  }
  //If the solution is found, stop the controller to avoid instabilities
  if (Math.abs(error) < 0.004 && absoluteRotationalSpeed <= 2)
  {
    Ki = 0;
    Kp = 0;
    Kd = 0;
  }
  //PID control itself
  float actuation = 2 * Kp * error
                  + Kd * (rotationalSpeed / 1800.)
                  + Ki * attitudeErrorIntegral;
  //If going against the wheel, increase actuation to magnify results
  if (actuation < 0)
  {
    actuation *= 3;
  }
  attitudeErrorIntegral += error;
  //Selection of preferred actuation
  if (useWheel)
  {
    targetWheelSpeed = constrain(wheelSpeed + actuation, -8000, 8000);
    runSetWheelSpeed();
  }
  else
  {
    targetMagnetorquerDirection = (actuation < 0);
    runMaximumMagneticTorque();
  }
}

void ESATADCS::runDemagnetize()
{
  Magnetorquer.writeEnable(true);
  for (unsigned int i = 0; i < demagnetizationIterations; i++)
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

void ESATADCS::runFollowMagnetometer()
{
  runAttitudeControlLoop(magneticAngle);
}

void ESATADCS::runFollowSun()
{
  runAttitudeControlLoop(sunAngle);
}

void ESATADCS::runMaximumMagneticTorque()
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

void ESATADCS::runSetWheelSpeed()
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

void ESATADCS::update()
{
  readSensors();
  run();
}

ESATADCS ADCS;
