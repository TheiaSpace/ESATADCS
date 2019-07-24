/*
 * Copyright (C) 2019 Theia Space, Universidad Politécnica de Madrid
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

#include "ESAT_ADCS-actuators/ESAT_Wheel.h"
#include "ESAT_ADCS-measurements/ESAT_Tachometer.h"
#include "ESAT_ADCS-run-modes/ESAT_WheelResetElectronicSpeedControllerRunMode.h"

byte ESAT_WheelResetElectronicSpeedControllerRunModeClass::identifier()
{
  return 0x22;
}

void ESAT_WheelResetElectronicSpeedControllerRunModeClass::run()
{
  if (pendingReset)
  {
    // Turn off wheel by switching its power off.
    ESAT_Wheel.switchOffElectronicSpeedController();
    // Poll for wheel to be stopped.
    if (ESAT_Tachometer.read() < wheelStoppedSpeedThreshold)
    {
      // Only resets if the wheel is stopped.
      delay(100);
      ESAT_Wheel.resetElectronicSpeedController();
      pendingReset = false;
    }
  }
}

ESAT_WheelResetElectronicSpeedControllerRunModeClass ESAT_WheelResetElectronicSpeedControllerRunMode;
