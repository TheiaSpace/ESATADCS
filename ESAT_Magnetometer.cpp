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

#include "ESAT_Magnetometer.h"
#include <ESAT_Util.h>
#include <Wire.h>

void ESAT_MagnetometerClass::begin()
{
  error = false;
  setBypassMode();
}

word ESAT_MagnetometerClass::getReading()
{
  Wire.beginTransmission(MAGNETOMETER_ADDRESS);
  Wire.write(READING_REGISTER);
  const byte writeStatus = Wire.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = Wire.requestFrom(int(MAGNETOMETER_ADDRESS), 4);
  if (bytesRead != 4)
  {
    error = true;
    return 0;
  }
  const byte xLowByte = Wire.read();
  const byte xHighByte = Wire.read();
  const byte yLowByte = Wire.read();
  const byte yHighByte = Wire.read();
  const word xFieldBits = word(xHighByte, xLowByte);
  const word yFieldBits = word(yHighByte, yLowByte);
  const int xField = ESAT_Util.wordToInt(xFieldBits);
  const int yField = ESAT_Util.wordToInt(yFieldBits);
  const int angle = round(atan2(xField, yField) * RAD_TO_DEG);
  if (angle < 0)
  {
    return angle + 360;
  }
  else
  {
    return angle;
  }
}

word ESAT_MagnetometerClass::read()
{
  setBypassMode();
  startReading();
  waitForReading();
  return getReading();
}

void ESAT_MagnetometerClass::setBypassMode()
{
  Wire.beginTransmission(CHIP_ADDRESS);
  Wire.write(BYPASS_REGISTER);
  Wire.write(ENABLE_BYPASS);
  const byte writeStatus = Wire.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
  }
}

void ESAT_MagnetometerClass::startReading()
{
  Wire.beginTransmission(MAGNETOMETER_ADDRESS);
  Wire.write(CONTROL_REGISTER);
  Wire.write(SINGLE_MEASUREMENT_MODE);
  const byte writeStatus = Wire.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
  }
}

void ESAT_MagnetometerClass::waitForReading()
{
  const byte timeout = 255;
  for (int i = 0; i < timeout; i++)
  {
    Wire.beginTransmission(MAGNETOMETER_ADDRESS);
    Wire.write(DATA_STATUS_REGISTER);
    const byte writeStatus = Wire.endTransmission();
    if (writeStatus != 0)
    {
      error = true;
      return;
    }
    const byte bytesRead = Wire.requestFrom(int(MAGNETOMETER_ADDRESS), 1);
    if (bytesRead != 1)
    {
      error = true;
      return;
    }
    const byte readingState = Wire.read();
    if ((readingState & DATA_READY) != 0)
    {
      return;
    }
  }
  error = true;
}

ESAT_MagnetometerClass ESAT_Magnetometer;
