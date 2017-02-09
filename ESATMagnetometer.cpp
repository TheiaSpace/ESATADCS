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
#include <ESATMath.h>

void ESATMagnetometer::begin()
{
  setBypassMode();
}

int ESATMagnetometer::getReading()
{
  byte rawReading[6];
  (void) I2C.read(magnetometerAddress,
                  readingRegister,
                  rawReading,
                  sizeof(rawReading));
  const int mx = *(reinterpret_cast<int*>(&(rawReading[0]))) ;
  const int my = *(reinterpret_cast<int*>(&(rawReading[2]))) ;
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
  (void) I2C.write(chipAddress, bypassRegister, enableBypass);
}

void ESATMagnetometer::startReading()
{
  (void) I2C.write(magnetometerAddress,
                   controlRegister,
                   singleMeasurementMode);
}

void ESATMagnetometer::waitForReading()
{
  byte timeout = 255;
  byte readingState;
  do
  {
    (void) I2C.read(magnetometerAddress,
                    dataStatusRegister,
                    &readingState,
                    sizeof(readingState));
  }
  while (!(readingState & dataReady) && timeout--);
}

ESATMagnetometer Magnetometer;
