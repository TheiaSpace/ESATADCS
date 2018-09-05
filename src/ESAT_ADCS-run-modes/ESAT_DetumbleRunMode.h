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

#ifndef ESAT_DetumbleRunMode_h
#define ESAT_DetumbleRunMode_h

#include <Arduino.h>
#include "ESAT_ADCS-run-modes/ESAT_ADCSRunMode.h"

// Run mode for detumbling (stopping the rotation of the satellite).
// Use the public instance ESAT_DetumbleRunMode.
class ESAT_DetumbleRunModeClass: public ESAT_ADCSRunMode
{
  public:
    // Return the identifier number of the mode.  Each run mode has
    // its own unique identifier number.
    byte identifier();

    // Use the magnetorquers to apply the maximum possible torque
    // against the angular velocity of the satellite.
    void run();
};

// Global instance of the detumbling run mode library.
extern ESAT_DetumbleRunModeClass ESAT_DetumbleRunMode;

#endif /* ESAT_DetumbleRunMode_h */
