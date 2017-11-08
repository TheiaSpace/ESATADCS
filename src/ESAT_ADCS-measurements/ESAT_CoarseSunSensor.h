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

#ifndef ESAT_CoarseSunSensor_h
#define ESAT_CoarseSunSensor_h

#include <Arduino.h>

// Coarse sun sensor used for attitude determination.
// Use the global instance ESAT_CoarseSunSensor.
//
// The coarse sun sensor consists of 4 light detectors,
// one on each side panel.  By comparing the readings
// of these 4 light detector, the coarse sun sensor library
// can estimate the direction of the light source.
class ESAT_CoarseSunSensorClass
{
  public:
    // Set up the coarse sun sensor.
    void begin();

    // Read the attitude provided by the coarse sun sensors:
    // the angle of the Sun direction measured counterclockwise from
    // the satellite's +X axis, from 0 degrees to 359 degrees.
    word readSunAngle();

    // Return the raw reading of the -X sun sensor as a percentage
    // from 0 % (dark) to 100 % (saturated).
    float readXMinus();

    // Return the raw reading of the +X sun sensor as a percentage
    // from 0 % (dark) to 100 % (saturated).
    float readXPlus();

    // Return the raw reading of the -Y sun sensor as a percentage
    // from 0 % (dark) to 100 % (saturated).
    float readYMinus();

    // Return the raw reading of the +Y sun sensor as a percentage
    // from 0 % (dark) to 100 % (saturated).
    float readYPlus();

  private:
    // Range of measurement of the analog converter.
    static const word MEASUREMENT_RANGE = 4095;

    // The individual sensors are connected to these pins.
    static const int PIN_X_PLUS = CSSXPLUS;
    static const int PIN_X_MINUS = CSSXMINUS;
    static const int PIN_Y_PLUS = CSSYPLUS;
    static const int PIN_Y_MINUS = CSSYMINUS;
};

// Global instance of the coarse sun sensor library.
extern ESAT_CoarseSunSensorClass ESAT_CoarseSunSensor;

#endif /* ESAT_CoarseSunSensor_h */
