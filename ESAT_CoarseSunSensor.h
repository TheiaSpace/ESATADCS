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
class ESAT_CoarseSunSensorClass
{
  public:
    // Set up the coarse sun sensor.
    void begin();

    // Read the attitude provided by the coarse sun sensors.
    int read();

  private:
    // The individual sensors are connected to these pins.
    static const int PIN_X_PLUS = CSSXPLUS;
    static const int PIN_X_MINUS = CSSXMINUS;
    static const int PIN_Y_PLUS = CSSYPLUS;
    static const int PIN_Y_MINUS = CSSYMINUS;
};

extern ESAT_CoarseSunSensorClass ESAT_CoarseSunSensor;

#endif /* ESAT_CoarseSunSensor_h */
