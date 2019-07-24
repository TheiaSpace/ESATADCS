/*
 * Copyright (C) 2017, 2018, 2019 Theia Space, Universidad Politécnica de Madrid
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
#ifdef ARDUINO_ESAT_ADCS
#include <EEPROM.h>
#endif /* ARDUINO_ESAT_ADCS */
#include <ESAT_Util.h>
#ifdef ARDUINO_ESAT_OBC
#include <SD.h>
#endif /* ARDUINO_ESAT_OBC */

#ifdef ARDUINO_ESAT_OBC
const char ESAT_GyroscopeClass::BIAS_FILENAME[] = "gyrobias";
#endif /* ARDUINO_ESAT_OBC */

void ESAT_GyroscopeClass::begin(const byte fullScaleConfiguration)
{
  error = false;
  configureLowPassFilter();
  configureRange(fullScaleConfiguration);
  setGain(fullScaleConfiguration);
  readBiasCorrection();
}

void ESAT_GyroscopeClass::configureBiasCorrection()
{
  const word samples = 1024;
  long average = 0;
  for (unsigned long sample = 0; sample < samples; sample = sample + 1)
  {
    average = average + readRawSample();
  }
  bias = average / samples;
  writeBiasCorrection();
}

void ESAT_GyroscopeClass::configureLowPassFilter()
{
  bus.beginTransmission(ADDRESS);
  bus.write(LOW_PASS_FILTER_CONFIGURATION_REGISTER);
  bus.write(LOW_PASS_FILTER_CONFIGURATION);
  const byte writeStatus = bus.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
  }
}

void ESAT_GyroscopeClass::configureRange(const byte fullScaleConfiguration)
{
  const byte fullScaleConfigurationOffset = 3;
  const byte configuration =
    fullScaleConfiguration << fullScaleConfigurationOffset;
  bus.beginTransmission(ADDRESS);
  bus.write(CONFIGURATION_REGISTER);
  bus.write(configuration);
  const byte writeStatus = bus.endTransmission();
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
  return (averageRawReading - bias) / gain;
}

int ESAT_GyroscopeClass::readRawSample()
{
  bus.beginTransmission(ADDRESS);
  bus.write(GYROSCOPE_READING_REGISTER);
  const byte writeStatus = bus.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = bus.requestFrom(int(ADDRESS), 2);
  if (bytesRead != 2)
  {
    error = true;
    return 0;
  }
  const byte highByte = bus.read();
  const byte lowByte = bus.read();
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

void ESAT_GyroscopeClass::readBiasCorrection()
{
#ifdef ARDUINO_ESAT_ADCS
  const byte highByte = EEPROM.read(BIAS_EEPROM_ADDRESS);
  const byte lowByte = EEPROM.read(BIAS_EEPROM_ADDRESS + 1);
  const word bits = word(highByte, lowByte);
  bias = ESAT_Util.wordToInt(bits);
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  File file = SD.open(BIAS_FILENAME, FILE_READ);
  if (file.available() == 2)
  {
    const byte highByte = byte(file.read());
    const byte lowByte = byte(file.read());
    const word bits = word(highByte, lowByte);
    bias = ESAT_Util.wordToInt(bits);
  }
  file.close();
#endif /* ARDUINO_ESAT_OBC */
}

void ESAT_GyroscopeClass::writeBiasCorrection()
{
#ifdef ARDUINO_ESAT_ADCS
  const word bits = ESAT_Util.intToWord(bias);
  EEPROM.write(BIAS_EEPROM_ADDRESS, highByte(bits));
  EEPROM.write(BIAS_EEPROM_ADDRESS + 1, lowByte(bits));
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  File file = SD.open(BIAS_FILENAME, FILE_WRITE);
  (void) file.seek(0);
  const word bits = ESAT_Util.intToWord(bias);
  (void) file.write(highByte(bits));
  (void) file.write(lowByte(bits));
  file.close();
#endif /* ARDUINO_ESAT_OBC */
}

ESAT_GyroscopeClass ESAT_Gyroscope;
