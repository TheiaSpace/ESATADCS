/*
 * Copyright (C) 2017 Theia Space, Universidad Politécnica de Madrid
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

#include "ESAT_ADCS-controllers/ESAT_AttitudePIDController.h"
#include "ESAT_ADCS.h"
#include "ESAT_ADCS-controllers/ESAT_MagnetorquerController.h"
#include "ESAT_ADCS-controllers/ESAT_WheelPIDController.h"

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

void ESAT_AttitudePIDControllerClass::applyActuation()
{
  switch (actuator)
  {
    case MAGNETORQUER:
      applyMagnetorquerActuation();
      break;
    case WHEEL:
      applyWheelActuation();
      break;
    default:
      break;
  }
}

void ESAT_AttitudePIDControllerClass::applyMagnetorquerActuation()
{
  if (actuation > 0)
  {
    ESAT_MagnetorquerController.rotateClockwise();
  }
  if (actuation < 0)
  {
    ESAT_MagnetorquerController.rotateCounterclockwise();
  }
  if (actuation == 0)
  {
    ESAT_MagnetorquerController.stop();
  }
}

void ESAT_AttitudePIDControllerClass::applyWheelActuation()
{
  ESAT_WheelPIDController.loop(ESAT_WheelPIDController.readTargetSpeed()
                               + actuation);
}

void ESAT_AttitudePIDControllerClass::begin()
{
  actuator = DEFAULT_ACTUATOR;
  derivativeGain = DEFAULT_DERIVATIVE_GAIN;
  detumblingThreshold = DEFAULT_DETUMBLING_THRESHOLD;
  errorDeadband = DEFAULT_ERROR_DEADBAND;
  errorDerivativeDeadband = DEFAULT_ERROR_DERIVATIVE_DEADBAND;
  integralGain = DEFAULT_INTEGRAL_GAIN;
  proportionalGain = DEFAULT_PROPORTIONAL_GAIN;
  gyroscopeUsage = DEFAULT_GYROSCOPE_USAGE_VALUE;
  errorIntegral = 0;
  previousError = 0;
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

float ESAT_AttitudePIDControllerClass::computeErrorDerivative()
{
  const int errorDifference = angleDifference(error, previousError);
  return errorDifference / ESAT_ADCS.period();
}

void ESAT_AttitudePIDControllerClass::loop(const word currentAngle)
{
  updateError(currentAngle);
  updateErrorDerivative();
  updateErrorIntegral();
  updateActuation();
  applyActuation();
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

void ESAT_AttitudePIDControllerClass::updateErrorDerivative()
{
  if (gyroscopeUsage)
  {
    const ESAT_AttitudeStateVector attitudeStateVector =
      ESAT_ADCS.attitudeStateVector();
    errorDerivative = attitudeStateVector.rotationalSpeed;
  }
  else
  {
    errorDerivative = computeErrorDerivative();
  }
  previousError = error;
}

void ESAT_AttitudePIDControllerClass::updateErrorIntegral()
{
  errorIntegral = errorIntegral + error * ESAT_ADCS.period();
}

ESAT_AttitudePIDControllerClass ESAT_AttitudePIDController;
