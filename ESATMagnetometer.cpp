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
#include <ESATI2C.h>
#include <ESATI2CDevice.h>
#include <ESATMath.h>

void ESATMagnetometer::begin()
{
  setBypassMode();
}

int ESATMagnetometer::getReading()
{
  ESATI2CDevice device(Wire, magnetometerAddress);
  const int mx = device.readLittleEndianWord(readingXRegister);
  if (device.error)
  {
    alive = false;
    return 0;
  }
  const int my = device.readLittleEndianWord(readingYRegister);
  if (device.error)
  {
    alive = false;
    return 0;
  }
  alive = true;
  const int angle = round(Math.atan2(mx, my) * RAD_TO_DEG);
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
  ESATI2CDevice device(Wire, chipAddress);
  device.writeByte(bypassRegister, enableBypass);
  if (device.error)
  {
    alive = false;
  }
  else
  {
    alive = true;
  }
}

void ESATMagnetometer::startReading()
{
  ESATI2CDevice device(Wire, magnetometerAddress);
  device.writeByte(controlRegister, singleMeasurementMode);
  if (device.error)
  {
    alive = false;
  }
  else
  {
    alive = true;
  }
}

void ESATMagnetometer::waitForReading()
{
  ESATI2CDevice device(Wire, magnetometerAddress);
  byte timeout = 255;
  byte readingState;
  do
  {
    readingState = device.readByte(dataStatusRegister);
  }
  while (!(readingState & dataReady) && timeout-- && !device.error);
  if (device.error)
  {
    alive = false;
  }
  else
  {
    alive = true;
  }
}

ESATMagnetometer Magnetometer;
