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

#ifndef ESATGyroscope_h
#define ESATGyroscope_h

#include <Energia.h>

// Gyroscope used for measuring the angular speed.
class ESATGyroscope
{
  public:
    // True if the magnetometer is alive.
    boolean alive;

    // Set up the gyroscope.
    void begin();

    // Read the gyroscope.  Return the average of a number of samples.
    int read(unsigned int samples);

  private:
    static const byte address = 0x69;
    static const byte configuration = B00011000; // Full scale select:
                                                 // B11 (+2000 dps)
    static const byte configurationRegister = 27;
    static const float gain = 131.0f - 4.775f * configuration;
    static const byte gyroscopeReadingRegister = 67;

    // Set the range of measurement.
    void configureRange();

    // Read a raw sample.
    int readRawSample();
};

extern ESATGyroscope Gyroscope;

#endif
