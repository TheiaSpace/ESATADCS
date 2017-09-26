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
#include <Wire.h>

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
  Wire.beginTransmission(address);
  Wire.write(configurationRegister);
  Wire.write(configuration);
  const byte writeStatus = Wire.endTransmission();
  if (writeStatus == 0)
  {
    alive = true;
  }
  else
  {
    alive = false;
  }
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
  Wire.beginTransmission(address);
  Wire.write(gyroscopeReadingRegister);
  const byte writeStatus = Wire.endTransmission();
  if (writeStatus != 0)
  {
    alive = false;
    return 0;
  }
  const byte bytesRead = Wire.requestFrom(int(address), 2);
  if (bytesRead != 2)
  {
    alive = false;
    return 0;
  }
  alive = true;
  const byte highByte = Wire.read();
  const byte lowByte = Wire.read();
  return word(highByte, lowByte);
}

void ESATGyroscope::setGain(const byte fullScaleConfiguration)
{
  const word fullScaleTable[] = { 250, 500, 1000, 2000 };
  const word fullScale = fullScaleTable[fullScaleConfiguration];
  gain = 32768.0 / fullScale;
}

ESATGyroscope Gyroscope;
