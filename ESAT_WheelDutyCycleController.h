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

#ifndef ESAT_WheelControlLoop_h
#define ESAT_WheelControlLoop_h

#include <Arduino.h>
#include "ESAT_ADCSRunMode.h"

// Wheel duty cycle controller.
// Just set the duty cycle of the wheel.
// Use the global instance ESAT_WheelDutyCycleController.
class ESAT_WheelDutyCycleControllerClass: public ESAT_ADCSRunMode
{
  public:
    // Target duty cycle of the wheel in percent points from -100 % to +100%.
    float dutyCycle;

    // Perform one iteration of the control loop.
    void loop(ESAT_AttitudeStateVector attitudeStateVector);
};

// Global instance of the wheel duty cycle controller library.
extern ESAT_WheelDutyCycleControllerClass ESAT_WheelDutyCycleController;

#endif /* ESAT_WheelDutyCycleController_h */
