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

#ifndef ESAT_MagnetorquerController_h
#define ESAT_MagnetorquerController_h

#include <Arduino.h>
#include "ESAT_ADCS-actuators/ESAT_Magnetorquer.h"

// Magnetorquer controller for applying a torque in a given direction.
// Use the public instance ESAT_MagnetorquerController.
class ESAT_MagnetorquerControllerClass
{
  public:
    // Apply a torque to rotate in the clockwise direction.
    void rotateClockwise();

    // Apply a torque to rotate in the counterclockwise direction.
    void rotateCounterclockwise();

    // Stop the magnetorquers.
    void stop();

  private:
    // Return the X-axis magnetorquer polarity needed for clockwise
    // rotation.
    ESAT_MagnetorquerClass::Polarity clockwiseXPolarity();

    // Return the Y-axis magnetorquer polarity needed for clockwise
    // rotation.
    ESAT_MagnetorquerClass::Polarity clockwiseYPolarity();

    // Return the X-axis magnetorquer polarity needed for
    // counterclockwise rotation.
    ESAT_MagnetorquerClass::Polarity counterclockwiseXPolarity();

    // Return the Y-axis magnetorquer polarity needed for
    // counterclockwise rotation.
    ESAT_MagnetorquerClass::Polarity counterclockwiseYPolarity();
};

// Global instance of the magnetorquer controller library.
extern ESAT_MagnetorquerControllerClass ESAT_MagnetorquerController;

#endif /* ESAT_MagnetorquerController_h */
