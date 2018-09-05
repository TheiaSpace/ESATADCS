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

#ifndef ESAT_WheelSetDutyCycleRunMode_h
#define ESAT_WheelSetDutyCycleRunMode_h

#include <Arduino.h>
#include "ESAT_ADCS-run-modes/ESAT_ADCSRunMode.h"

// Wheel duty cycle controller.
// Use the global instance ESAT_WheelSetDutyCycleRunMode.
class ESAT_WheelSetDutyCycleRunModeClass: public ESAT_ADCSRunMode
{
  public:
    // Target duty cycle of the wheel in percent points from -100 % to +100%.
    float dutyCycle;

    // Return the identifier number of the mode.  Each run mode has
    // its own unique identifier number.
    byte identifier();

    // Perform one iteration of the control loop.
    void run();
};

// Global instance of the wheel duty cycle controller library.
extern ESAT_WheelSetDutyCycleRunModeClass ESAT_WheelSetDutyCycleRunMode;

#endif /* ESAT_WheelSetDutyCycleRunMode_h */
