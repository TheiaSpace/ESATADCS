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

#ifndef ESAT_FollowMagneticTargetRunMode_h
#define ESAT_FollowMagneticTargetRunMode_h

#include <Arduino.h>
#include "ESAT_ADCS-run-modes/ESAT_ADCSRunMode.h"

// Run mode for controlling the attitude to follow a magnetic target angle.
// Use the public instance ESAT_FollowMagneticTargetRunMode.
class ESAT_FollowMagneticTargetRunModeClass: public ESAT_ADCSRunMode
{
  public:
    // Return the identifier number of the mode.  Each run mode has
    // its own unique identifier number.
    byte identifier();

    // Keep a magnetic angle.
    void run();
};

// Global instance of the library of the run mode for controlling the
// magnetic attitude.
extern ESAT_FollowMagneticTargetRunModeClass ESAT_FollowMagneticTargetRunMode;

#endif /* ESAT_FollowMagneticTargetRunMode_h */
