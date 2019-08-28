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

#ifndef ESAT_WheelResetElectronicSpeedControllerRunMode_h
#define ESAT_WheelResetElectronicSpeedControllerRunMode_h

#include <Arduino.h>
#include "ESAT_ADCS-run-modes/ESAT_ADCSRunMode.h"

// Run mode for resetting the wheel electronic speed controller.
// Use the public instance ESAT_WheelResetElectronicSpeedControllerRunMode.
class ESAT_WheelResetElectronicSpeedControllerRunModeClass: public ESAT_ADCSRunMode
{
  public:
    // Flag that indicates if a reset is pending or not.  This is
    // necessary because a run mode is always executing its run()
    // function in a loop and we only want to perform a single reset.
    boolean pendingReset;

    // Return the identifier number of the mode.  Each run mode has
    // its own unique identifier number.
    byte identifier();

    // Reset the wheel electronic speed controller.
    void run();

  private:
    // Number of wheel revolutions per minute below which the wheel is
    // considered to be stopped.
    const word wheelStoppedSpeedThreshold = 10;
};

// Global instance of the library of the run mode for resetting the
// wheel electronic speed controller.
extern ESAT_WheelResetElectronicSpeedControllerRunModeClass ESAT_WheelResetElectronicSpeedControllerRunMode;

#endif /* ESAT_WheelResetElectronicSpeedControllerRunMode_h */
