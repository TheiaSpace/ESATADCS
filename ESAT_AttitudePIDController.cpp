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

#include "ESAT_AttitudePIDController.h"
#include "ESAT_MagnetorquerApplyMaximumTorqueRunMode.h"
#include "ESAT_WheelPIDController.h"

boolean ESAT_AttitudePIDControllerClass::aboveDetumblingThreshold()
{
  if (abs(errorDerivative) > detumblingThreshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int ESAT_AttitudePIDControllerClass::angleDifference(const int minuend,
                                                     const int subtrahend)
{
  const int difference = minuend - subtrahend;
  if (difference > 180)
  {
    return difference - 360;
  }
  if (difference < -180)
  {
    return difference + 360;
  }
  return difference;
}

void ESAT_AttitudePIDControllerClass::applyActuation(const ESAT_AttitudeStateVector attitudeStateVector)
{
  switch (actuator)
  {
    case MAGNETORQUER:
      applyMagnetorquerActuation(attitudeStateVector);
      break;
    case WHEEL:
      applyWheelActuation(attitudeStateVector);
      break;
    default:
      break;
  }
}

void ESAT_AttitudePIDControllerClass::applyMagnetorquerActuation(const ESAT_AttitudeStateVector attitudeStateVector)
{
  if (actuation > 0)
  {
    ESAT_MagnetorquerApplyMaximumTorqueRunMode.mode =
      ESAT_MagnetorquerApplyMaximumTorqueRunMode.ROTATE_CLOCKWISE;
  }
  if (actuation < 0)
  {
    ESAT_MagnetorquerApplyMaximumTorqueRunMode.mode =
      ESAT_MagnetorquerApplyMaximumTorqueRunMode.ROTATE_COUNTERCLOCKWISE;
  }
  if (actuation == 0)
  {
    ESAT_MagnetorquerApplyMaximumTorqueRunMode.mode =
      ESAT_MagnetorquerApplyMaximumTorqueRunMode.STOP;
  }
  ESAT_MagnetorquerApplyMaximumTorqueRunMode.loop(attitudeStateVector);
}

void ESAT_AttitudePIDControllerClass::applyWheelActuation(const ESAT_AttitudeStateVector attitudeStateVector)
{
  ESAT_WheelPIDController.loop(attitudeStateVector.wheelSpeed + actuation,
                               attitudeStateVector.wheelSpeed);
}

void ESAT_AttitudePIDControllerClass::begin(const float periodInSeconds)
{
  actuator = DEFAULT_ACTUATOR;
  derivativeGain = DEFAULT_DERIVATIVE_GAIN;
  detumblingThreshold = DEFAULT_DETUMBLING_THRESHOLD;
  errorDeadband = DEFAULT_ERROR_DEADBAND;
  errorDerivativeDeadband = DEFAULT_ERROR_DERIVATIVE_DEADBAND;
  integralGain = DEFAULT_INTEGRAL_GAIN;
  proportionalGain = DEFAULT_PROPORTIONAL_GAIN;
  useGyroscope = DEFAULT_USE_GYROSCOPE_VALUE;
  period = periodInSeconds;
  errorIntegral = 0;
  oldError = 0;
  period = periodInSeconds;
}

boolean ESAT_AttitudePIDControllerClass::belowDeadbandThreshold()
{
  if ((abs(error) < errorDeadband)
      && (abs(errorDerivative) < errorDerivativeDeadband))
  {
    return true;
  }
  else
  {
    return false;
  }
}

float ESAT_AttitudePIDControllerClass::computeActuation()
{
  if (belowDeadbandThreshold())
  {
    return 0;
  }
  if (aboveDetumblingThreshold())
  {
    return derivativeGain * errorDerivative;
  }
  return proportionalGain * error
    + integralGain * errorIntegral
    + derivativeGain * errorDerivative;
}

void ESAT_AttitudePIDControllerClass::loop(const word currentAngle,
                                           const ESAT_AttitudeStateVector attitudeStateVector)
{
  updateError(currentAngle);
  updateErrorDerivative(attitudeStateVector.rotationalSpeed);
  updateActuation();
  updateErrorIntegral();
  applyActuation(attitudeStateVector);
}

void ESAT_AttitudePIDControllerClass::resetErrorIntegral()
{
  errorIntegral = 0;
}

void ESAT_AttitudePIDControllerClass::updateActuation()
{
  actuation = computeActuation();
}

void ESAT_AttitudePIDControllerClass::updateError(const word currentAngle)
{
  error = angleDifference(targetAngle, currentAngle);
}

float ESAT_AttitudePIDControllerClass::computeErrorDerivative()
{
  const int errorDifference = angleDifference(error, oldError);
  return errorDifference / period;
}

void ESAT_AttitudePIDControllerClass::updateErrorDerivative(const int rotationalSpeed)
{
  if (useGyroscope)
  {
    errorDerivative = rotationalSpeed;
  }
  else
  {
    errorDerivative = computeErrorDerivative();
  }
  oldError = error;
}

void ESAT_AttitudePIDControllerClass::updateErrorIntegral()
{
  errorIntegral = errorIntegral + error * period;
}

ESAT_AttitudePIDControllerClass ESAT_AttitudePIDController;
