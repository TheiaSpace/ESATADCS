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

#ifndef ESATTachometer_h
#define ESATTachometer_h

#include <Arduino.h>

// Tachometer used for reading the speed of the rotating wheel.
class ESATTachometer
{
  public:
    // Start the tachometer.
    void begin();

    // Read the speed of the rotating wheel (in revolutions per minute).
    unsigned int read();

  private:
    volatile unsigned int count;
    unsigned int previousCount;
    unsigned long previousReadingTime;

    // The rotating wheel has several marks, so the tachometer gives
    // as many counts per revolution.
    static const unsigned int COUNTS_PER_REVOLUTION = 8;

    // The output signal of the tachometer goes to this pin.
    static const unsigned int PIN = TCH;

    // Increment the counter of the tachometer.
    static void incrementCounter();
};

extern ESATTachometer Tachometer;

#endif
