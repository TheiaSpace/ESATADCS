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

#include "ESAT_WheelPIDController.h"
#include "ESAT_Wheel.h"

void ESAT_WheelPIDControllerClass::begin(const float periodInSeconds)
{
  derivativeGain = DEFAULT_DERIVATIVE_GAIN;
  integralGain = DEFAULT_INTEGRAL_GAIN;
  proportionalGain = DEFAULT_PROPORTIONAL_GAIN;
  period = periodInSeconds;
  errorIntegral = 0;
  oldError = 0;
}

void ESAT_WheelPIDControllerClass::loop(const ESAT_AttitudeStateVector attitudeStateVector)
{
  const int error = int(targetSpeed) - int(attitudeStateVector.wheelSpeed);
  errorIntegral = errorIntegral + error * period;
  const float errorDerivative = (error - oldError) / period;
  oldError = error;
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

void ESAT_WheelPIDControllerClass::resetErrorIntegral()
{
  errorIntegral = 0;
}

ESAT_WheelPIDControllerClass ESAT_WheelPIDController;
