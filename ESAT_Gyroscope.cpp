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

#include "ESAT_Gyroscope.h"
#include <Wire.h>

void ESAT_GyroscopeClass::begin(const byte fullScaleConfiguration)
{
  error = false;
  configureRange(fullScaleConfiguration);
  setGain(fullScaleConfiguration);
}

void ESAT_GyroscopeClass::configureRange(const byte fullScaleConfiguration)
{
  const byte fullScaleConfigurationOffset = 3;
  const byte configuration =
    fullScaleConfiguration << fullScaleConfigurationOffset;
  Wire.beginTransmission(ADDRESS);
  Wire.write(CONFIGURATION_REGISTER);
  Wire.write(configuration);
  const byte writeStatus = Wire.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
  }
}

int ESAT_GyroscopeClass::read(unsigned int samples)
{
  long cumulativeRawReading = 0;
  for (unsigned int i = 0; i < samples; i++)
  {
    cumulativeRawReading = cumulativeRawReading + readRawSample();
  }
  const long averageRawReading = cumulativeRawReading / samples;
  return averageRawReading / gain;
}

int ESAT_GyroscopeClass::readRawSample()
{
  Wire.beginTransmission(ADDRESS);
  Wire.write(GYROSCOPE_READING_REGISTER);
  const byte writeStatus = Wire.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = Wire.requestFrom(int(ADDRESS), 2);
  if (bytesRead != 2)
  {
    error = true;
    return 0;
  }
  const byte highByte = Wire.read();
  const byte lowByte = Wire.read();
  return word(highByte, lowByte);
}

void ESAT_GyroscopeClass::setGain(const byte fullScaleConfiguration)
{
  const word fullScaleTable[] = { 250, 500, 1000, 2000 };
  const word fullScale = fullScaleTable[fullScaleConfiguration];
  gain = 32768.0 / fullScale;
}

ESAT_GyroscopeClass ESAT_Gyroscope;
