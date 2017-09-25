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

#include "ESATWheel.h"
#include <ESATI2CDevice.h>
#include <MspFlash.h>

#define flash SEGMENT_D

void ESATWheel::begin()
{
  pinMode(pin, OUTPUT);
  loadCalibration();
  electronicSpeedController.attach(pin);
  delay(1000);
  programElectronicSpeedController();
}

void ESATWheel::defaultCalibration()
{
  calibration[0] = 128.0;
  calibration[1] = 1.9e-2;
  calibration[2] = 0;
}

void ESATWheel::loadCalibration()
{
  Flash.read(flash,
             reinterpret_cast<unsigned char*>(calibration),
             sizeof(calibration));
  if (calibration[0] < 1)
  {
    defaultCalibration();
  }
}

void ESATWheel::saveCalibration()
{
  Flash.erase(flash);
  Flash.write(flash,
              reinterpret_cast<unsigned char*>(calibration),
              sizeof(calibration));
}

void ESATWheel::write(int rpm)
{
  const float unconstrainedDutyCycle = 
    (calibration[0] + rpm * (calibration[1] + rpm * calibration[2]));
  const byte dutyCycle =
          constrain(round(unconstrainedDutyCycle), 0, 255);
  writeDutyCycle(dutyCycle);
}

void ESATWheel::writeDutyCycle(byte dutyCycle)
{
  const unsigned int microseconds = map(dutyCycle, 0, 255, minimum, maximum);
  electronicSpeedController.writeMicroseconds(microseconds);
}

void ESATWheel::programElectronicSpeedController()
{
  // Perform the ESC programming sequence (high, low and medium again)
  writeDutyCycle(255);
  ESATI2CDevice device(Wire, programmingAddress);
  device.writeByte(programmingRegister, programmingMessage);
  delay(1000);
  delay(1000);
  writeDutyCycle(0);
  delay(1000);
  writeDutyCycle(128);
  delay(1000);
}

ESATWheel Wheel;
