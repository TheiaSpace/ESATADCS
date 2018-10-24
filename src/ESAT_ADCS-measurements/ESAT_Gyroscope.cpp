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

#include "ESAT_ADCS-measurements/ESAT_Gyroscope.h"
#include <ESAT_Util.h>

void ESAT_GyroscopeClass::begin(const byte fullScaleConfiguration)
{
  error = false;
#ifdef ARDUINO_ESAT_ADCS
  bus = &WireADCS;
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  bus = &WireOBC;
#endif /* ARDUINO_ESAT_OBC */
  configureRange(fullScaleConfiguration);
  setGain(fullScaleConfiguration);
}

void ESAT_GyroscopeClass::configureRange(const byte fullScaleConfiguration)
{
  const byte fullScaleConfigurationOffset = 3;
  const byte configuration =
    fullScaleConfiguration << fullScaleConfigurationOffset;
  bus->beginTransmission(ADDRESS);
  bus->write(CONFIGURATION_REGISTER);
  bus->write(configuration);
  const byte writeStatus = bus->endTransmission();
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
  const long averageRawReading = cumulativeRawReading / long(samples);
  return averageRawReading / gain;
}

int ESAT_GyroscopeClass::readRawSample()
{
  bus->beginTransmission(ADDRESS);
  bus->write(GYROSCOPE_READING_REGISTER);
  const byte writeStatus = bus->endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = bus->requestFrom(int(ADDRESS), 2);
  if (bytesRead != 2)
  {
    error = true;
    return 0;
  }
  const byte highByte = bus->read();
  const byte lowByte = bus->read();
  const word bits = word(highByte, lowByte);
  const int reading = ESAT_Util.wordToInt(bits);
#ifdef ARDUINO_ESAT_OBC
  return reading;
#endif /* ARDUINO_ESAT_OBC */
#ifdef ARDUINO_ESAT_ADCS
  return -reading;
#endif /* ARDUINO_ESAT_ADCS */
}

void ESAT_GyroscopeClass::setGain(const byte fullScaleConfiguration)
{
  const word fullScaleTable[] = { 250, 500, 1000, 2000 };
  const word fullScale = fullScaleTable[fullScaleConfiguration];
  gain = 32768.0 / fullScale;
}

ESAT_GyroscopeClass ESAT_Gyroscope;
