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

void ESATGyroscope::begin(const byte fullScaleConfiguration)
{
  configureRange(fullScaleConfiguration);
  setGain(fullScaleConfiguration);
}

void ESATGyroscope::configureRange(const byte fullScaleConfiguration)
{
  const byte fullScaleConfigurationOffset = 3;
  const byte configuration =
    fullScaleConfiguration << fullScaleConfigurationOffset;
  const byte errorCode =
    I2C.write(address, configurationRegister, configuration);
  alive = (errorCode == 0);
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
  const byte errorCode =
    I2C.read(address, gyroscopeReadingRegister, buffer, sizeof(buffer));
  alive = (errorCode == 0);
  if (alive)
  {
    return (buffer[4] << 8) | buffer[5];
  }
  else
  {
    return 0;
  }
}

void ESATGyroscope::setGain(const byte fullScaleConfiguration)
{
  const byte fullScaleConfigurationOffset = 3;
  const byte configuration =
    fullScaleConfiguration << fullScaleConfigurationOffset;
  gain = 131.0f - 4.775f * configuration;
}

ESATGyroscope Gyroscope;
