/*
 * Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_ADCSLED_h
#define ESAT_ADCSLED_h

#include <Arduino.h>

// Heartbeat LED of the ADCS board.
// Use the global instance ESAT_ADCSLED.
//
// The ESAT ADCS board has an LED that proves that the board is working.
class ESAT_ADCSLEDClass
{
  public:
    // Set up the LED.
    void begin();

    // Write an brightness value (from 0 to 100 %) to the LED.
    // Negative values turn off the LED.
    // Values above 100 % work the same as 100 %.
    void write(float brightness);

  private:
#ifdef ARDUINO_ESAT_ADCS
    // Pin number of the LED control line.
    static const int LED_CONTROL_LINE = LED_A;
#endif /* ARDUINO_ESAT_ADCS */
};

// Global instance of the ADCS LED library.
extern ESAT_ADCSLEDClass ESAT_ADCSLED;

#endif /* ESAT_ADCSLED_h */
