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

#ifndef ESAT_ADCSRunMode_h
#define ESAT_ADCSRunMode_h

#include <Arduino.h>

// An ADCS run mode has a run() method that performs whatever actions
// are relevant to the mode (for example, an attitude control loop
// will command the actuators to control the attitude of the
// satellite).
class ESAT_ADCSRunMode
{
  public:
    virtual ~ESAT_ADCSRunMode() {};

    // Return the identifier number of the mode.  Each run mode has
    // its own unique identifier number.
    virtual byte identifier() = 0;

    // Perform one iteration of the ADCS run mode.
    virtual void run() = 0;
};

#endif /* ESAT_ADCSRunMode_h */
