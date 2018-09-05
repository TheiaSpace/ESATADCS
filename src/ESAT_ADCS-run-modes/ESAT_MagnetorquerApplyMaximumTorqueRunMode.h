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

#ifndef ESAT_MagnetorquerApplyMaximumTorqueRunMode_h
#define ESAT_MagnetorquerApplyMaximumTorqueRunMode_h

#include <Arduino.h>
#include "ESAT_ADCS-run-modes/ESAT_ADCSRunMode.h"

// Magnetorquer maximum torque mode.
// Use the public instance ESAT_MagnetorquerApplyMaximumTorqueRunMode.
class ESAT_MagnetorquerApplyMaximumTorqueRunModeClass: public ESAT_ADCSRunMode
{
  public:
    // Available modes of operation.
    enum Mode
    {
      ROTATE_CLOCKWISE,
      ROTATE_COUNTERCLOCKWISE
    };

    // Current mode of operation.
    Mode mode;

    // Return the identifier number of the mode.  Each run mode has
    // its own unique identifier number.
    byte identifier();

    // Perform one iteration of the maximum magnetic torque mode:
    // apply the maximum possible torque towards the specified direction
    // given the attitude of the satellite.
    void run();
};

// Global instance of the magnetorquer maximum torque run mode library.
extern ESAT_MagnetorquerApplyMaximumTorqueRunModeClass ESAT_MagnetorquerApplyMaximumTorqueRunMode;

#endif /* ESAT_MagnetorquerApplyMaximumTorquerRunMode_h */
