/*
 * Copyright (C) 2017, 2018 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_Tachometer_h
#define ESAT_Tachometer_h

#include <Arduino.h>

// Tachometer used for reading the speed of the rotating wheel.
// Use the global instance ESAT_Tachometer.
//
// The wheel has a series of marks; the tachometer hardware
// sends a pulse every time a mark passes above it so that
// the tachometer library can count these pulses to estimate
// the wheel speed.
class ESAT_TachometerClass
{
  public:
    // Start the tachometer.
    void begin();

    // Read the speed of the rotating wheel (in revolutions per minute).
    // This works by counting the number of tachometer pulses since
    // the latest call to read(), so calling read() too often will
    // lower the accuracty of the measurement.
    unsigned int read();

  private:
    // The rotating wheel has several marks, so the tachometer gives
    // as many counts per revolution.
    static const unsigned int COUNTS_PER_REVOLUTION = 8;

#ifdef ARDUINO_ESAT_ADCS
    // The output signal of the tachometer goes to this pin.
    static const unsigned int PIN = TCH_A;
#endif /* ARDUINO_ESAT_ADCS */

#ifdef ARDUINO_ESAT_OBC
    // The output signal of the tachometer goes to this pin.
    static const unsigned int PIN = TCH;
#endif /* ARDUINO_ESAT_OBC */

    // Number of tachometer pulses since the latest measurement.
    volatile unsigned int count;

    // Number of tachometer pulses at the previous measurement.
    unsigned int previousCount;

    // Processor uptime in milliseconds at the previous measurement.
    unsigned long previousReadingTime;

    // Increment the counter of the tachometer.
    static void incrementCounter();
};

// Global instance of the tachometer library.
extern ESAT_TachometerClass ESAT_Tachometer;

#endif /* ESAT_Tachometer_h */
