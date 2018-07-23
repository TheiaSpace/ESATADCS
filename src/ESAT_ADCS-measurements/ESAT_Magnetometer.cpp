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

#include "ESAT_ADCS-measurements/ESAT_Magnetometer.h"
#include <ESAT_Util.h>

void ESAT_MagnetometerClass::begin()
{
  error = false;
#ifdef ARDUINO_ESAT_ADCS
  bus = &Wire1;
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  bus = &Wire;
#endif /* ARDUINO_ESAT_OBC */
  setBypassMode();
}

word ESAT_MagnetometerClass::getReading()
{
  bus->beginTransmission(MAGNETOMETER_ADDRESS);
  bus->write(READING_REGISTER);
  const byte writeStatus = bus->endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = bus->requestFrom(int(MAGNETOMETER_ADDRESS), 4);
  if (bytesRead != 4)
  {
    error = true;
    return 0;
  }
  const byte xLowByte = bus->read();
  const byte xHighByte = bus->read();
  const byte yLowByte = bus->read();
  const byte yHighByte = bus->read();
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
  bus->beginTransmission(CHIP_ADDRESS);
  bus->write(BYPASS_REGISTER);
  bus->write(ENABLE_BYPASS);
  const byte writeStatus = bus->endTransmission();
  if (writeStatus != 0)
  {
    error = true;
  }
}

void ESAT_MagnetometerClass::startReading()
{
  bus->beginTransmission(MAGNETOMETER_ADDRESS);
  bus->write(CONTROL_REGISTER);
  bus->write(SINGLE_MEASUREMENT_MODE);
  const byte writeStatus = bus->endTransmission();
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
    bus->beginTransmission(MAGNETOMETER_ADDRESS);
    bus->write(DATA_STATUS_REGISTER);
    const byte writeStatus = bus->endTransmission();
    if (writeStatus != 0)
    {
      error = true;
      return;
    }
    const byte bytesRead = bus->requestFrom(int(MAGNETOMETER_ADDRESS), 1);
    if (bytesRead != 1)
    {
      error = true;
      return;
    }
    const byte readingState = bus->read();
    if ((readingState & DATA_READY) != 0)
    {
      return;
    }
  }
  error = true;
}

ESAT_MagnetometerClass ESAT_Magnetometer;
