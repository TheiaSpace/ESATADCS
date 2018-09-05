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

#ifndef ESAT_AttitudeStateVector_h
#define ESAT_AttitudeStateVector_h

#include <Arduino.h>

// Attitude state vector of the ESAT.
class ESAT_AttitudeStateVector
{
  public:
    // Angle of the magnetic North direction relative to the
    // satellite's +X axis, positive counterclockwise.
    // This is the same as the attitude of the satellite's +X axis
    // with respect to the magnetic North, positive clockwise.
    word magneticAngle;

    // Rotational speed of the satellite in degrees per second,
    // positive counterclockwise.
    int rotationalSpeed;

    // Angle of the sunwards direction relative to the satellite's +X
    // axis, positive counterclockwise.
    // This is the same as the attitude of the satellite's +X axis
    // with respect to the sunwards direction, positive clockwise.
    word sunAngle;

    // Rotational speed of the wheel in revolutions per minute.
    word wheelSpeed;
};

#endif /* ESAT_AttitudeStateVector_h */
