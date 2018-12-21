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

#include "ESAT_ADCS-controllers/ESAT_WheelPIDController.h"
#include "ESAT_ADCS.h"
#include "ESAT_ADCS-actuators/ESAT_Wheel.h"

void ESAT_WheelPIDControllerClass::begin()
{
  derivativeGain = DEFAULT_DERIVATIVE_GAIN;
  integralGain = DEFAULT_INTEGRAL_GAIN;
  proportionalGain = DEFAULT_PROPORTIONAL_GAIN;
  errorIntegral = 0;
  previousError = 0;
  targetSpeed = 0;
}

void ESAT_WheelPIDControllerClass::loop(const word newTargetSpeed)
{
  const float period = ESAT_ADCS.period();
  targetSpeed = newTargetSpeed;
  const ESAT_AttitudeStateVector attitudeStateVector =
    ESAT_ADCS.attitudeStateVector();
  const int error = int(targetSpeed) - int(attitudeStateVector.wheelSpeed);
  errorIntegral = errorIntegral + error * period;
  const float errorDerivative = (float(error) - float(previousError)) / period;
  previousError = error;
  const float control =
    proportionalGain * error
    + integralGain * errorIntegral
    + derivativeGain * errorDerivative;
  if (control > 0)
  {
    ESAT_Wheel.writeSpeed(control);
  }
  else
  {
    ESAT_Wheel.writeSpeed(0);
  }
}

word ESAT_WheelPIDControllerClass::readTargetSpeed()
{
  return targetSpeed;
}

void ESAT_WheelPIDControllerClass::resetErrorIntegral()
{
  errorIntegral = 0;
}

ESAT_WheelPIDControllerClass ESAT_WheelPIDController;
