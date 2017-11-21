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

#ifndef ESAT_DetumbleRunMode_h
#define ESAT_DetumbleRunMode_h

#include <Arduino.h>
#include "ESAT_ADCS-run-modes/ESAT_ADCSRunMode.h"

// Magnetorquer detumble mode.
// Use the public instance ESAT_DetumbleRunMode.
class ESAT_DetumbleRunModeClass: public ESAT_ADCSRunMode
{
  public:
    // Return the identifier number of the mode.  Each run mode has
    // its own unique identifier number.
    byte identifier();

    // Apply the maximum possible torque towards the specified direction
    // given the angular velocity of the satellite oposing to it.
    void run();
};

// Global instance of the magnetorquer maximum torque run mode library.
extern ESAT_DetumbleRunModeClass ESAT_DetumbleRunMode;

#endif /* ESAT_DetumbleRunMode_h */
