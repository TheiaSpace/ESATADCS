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

#include "ESATGyroscope.h"
#include <ESATI2C.h>

void ESATGyroscope::begin()
{
  configureRange();
}

void ESATGyroscope::configureRange()
{
  (void) I2C.write(address, configurationRegister, configuration);
}

int ESATGyroscope::read(unsigned int samples)
{
  long cumulativeRawReading = 0;
  for (unsigned int i = 0; i < samples; i++)
  {
    cumulativeRawReading = cumulativeRawReading + readRawSample();
  }
  const long averageRawReading = cumulativeRawReading / samples;
  return averageRawReading / gain;
}

int ESATGyroscope::readRawSample()
{
  byte buffer[6];
  (void) I2C.read(address, gyroscopeReadingRegister, buffer, sizeof(buffer));
  return (buffer[4] << 8) | buffer[5];
}

ESATGyroscope Gyroscope;