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

#include "ESATMagnetometer.h"
#include <Wire.h>

void ESATMagnetometer::begin()
{
  error = false;
  setBypassMode();
}

int ESATMagnetometer::getReading()
{
  Wire.beginTransmission(magnetometerAddress);
  Wire.write(readingRegister);
  const byte writeStatus = Wire.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = Wire.requestFrom(int(magnetometerAddress), 4);
  if (bytesRead != 4)
  {
    error = true;
    return 0;
  }
  const byte xLowByte = Wire.read();
  const byte xHighByte = Wire.read();
  const byte yLowByte = Wire.read();
  const byte yHighByte = Wire.read();
  const int mx = word(xHighByte, xLowByte);
  const int my = word(yHighByte, yLowByte);
  const int angle = round(atan2(mx, my) * RAD_TO_DEG) % 360;
  return angle;
}

int ESATMagnetometer::read()
{
  setBypassMode();
  startReading();
  waitForReading();
  return getReading();
}

void ESATMagnetometer::setBypassMode()
{
  Wire.beginTransmission(chipAddress);
  Wire.write(bypassRegister);
  Wire.write(enableBypass);
  const byte writeStatus = Wire.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
  }
}

void ESATMagnetometer::startReading()
{
  Wire.beginTransmission(magnetometerAddress);
  Wire.write(controlRegister);
  Wire.write(singleMeasurementMode);
  const byte writeStatus = Wire.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
  }
}

void ESATMagnetometer::waitForReading()
{
  const byte timeout = 255;
  for (int i = 0; i < timeout; i++)
  {
    Wire.beginTransmission(magnetometerAddress);
    Wire.write(dataStatusRegister);
    const byte writeStatus = Wire.endTransmission();
    if (writeStatus != 0)
    {
      error = true;
      return;
    }
    const byte bytesRead = Wire.requestFrom(int(dataStatusRegister), 1);
    if (bytesRead != 1)
    {
      error = true;
      return;
    }
    const byte readingState = Wire.read();
    if ((readingState & dataReady) != 0)
    {
      return;
    }
  }
  error = true;
}

ESATMagnetometer Magnetometer;
