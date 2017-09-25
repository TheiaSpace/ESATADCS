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
  const byte errorCode = I2C.write(chipAddress, bypassRegister, enableBypass);
  alive = (errorCode == 0);
}

void ESATMagnetometer::startReading()
{
  const byte errorCode = I2C.write(magnetometerAddress,
                                   controlRegister,
                                   singleMeasurementMode);
  alive = (errorCode == 0);
}

void ESATMagnetometer::waitForReading()
{
  byte timeout = 255;
  byte readingState;
  do
  {
    const byte errorCode = I2C.read(magnetometerAddress,
                                    dataStatusRegister,
                                    &readingState,
                                    sizeof(readingState));
    alive = (errorCode == 0);
  }
  while (!(readingState & dataReady) && timeout-- && alive);
}

ESATMagnetometer Magnetometer;
