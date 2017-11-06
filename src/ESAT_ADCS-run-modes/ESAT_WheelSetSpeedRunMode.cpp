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

#include "ESAT_ADCS-run-modes/ESAT_WheelSetSpeedRunMode.h"
#include "ESAT_ADCS-controllers/ESAT_WheelPIDController.h"

byte ESAT_WheelSetSpeedRunModeClass::identifier()
{
  return 0x21;
}

void ESAT_WheelSetSpeedRunModeClass::loop(const ESAT_AttitudeStateVector attitudeStateVector)
{
  ESAT_WheelPIDController.loop(targetSpeed, attitudeStateVector.wheelSpeed);
}

ESAT_WheelSetSpeedRunModeClass ESAT_WheelSetSpeedRunMode;
