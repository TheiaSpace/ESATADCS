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

#include "ESAT_ADCS-measurements/ESAT_Tachometer.h"
#ifdef ARDUINO_ESAT_ADCS
#include <Wire.h>
#endif /* ARDUINO_ESAT_ADCS */

void ESAT_TachometerClass::begin()
{
#ifdef ARDUINO_ESAT_OBC
  count = 0;
  previousCount = 0;
  previousReadingTime = 0;
  pinMode(PIN, INPUT_PULLUP);
  attachInterrupt(PIN, incrementCounter, FALLING);
#endif /* ARDUINO_ESAT_OBC */
}

#ifdef ARDUINO_ESAT_OBC
void ESAT_TachometerClass::incrementCounter()
{
  ESAT_Tachometer.count = ESAT_Tachometer.count + 1;
}
#endif /* ARDUINO_ESAT_OBC */

unsigned int ESAT_TachometerClass::read()
{
#ifdef ARDUINO_ESAT_ADCS
  const byte bytesToRead = 9;
  byte buffer[bytesToRead];
  const byte highByteIndex = 2;
  const byte lowByteIndex = 3;
  Wire1.beginTransmission(ELECTRONIC_SPEED_CONTROLLER_ADDRESS);
  (void) Wire1.write(WHEEL_SPEED_REGISTER);
  (void) Wire1.endTransmission();
  (void) Wire1.requestFrom(ELECTRONIC_SPEED_CONTROLLER_ADDRESS, bytesToRead);
  for (int index = 0; index < bytesToRead; index++)
  {
    buffer[index] = Wire1.read();
  }
  const unsigned int reading =
    word(buffer[highByteIndex], buffer[lowByteIndex]);
  return reading;
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  const unsigned long currentTime = millis();
  const unsigned long ellapsedMilliseconds = currentTime - previousReadingTime;
  previousReadingTime = currentTime;
  const unsigned long millisecondsPerMinute = 60000;
  const unsigned int reading =
    (long(millisecondsPerMinute / COUNTS_PER_REVOLUTION)
     * long(count - previousCount)) / ellapsedMilliseconds;
  previousCount = count;
  return reading;
#endif /* ARDUINO_ESAT_OBC */
}

ESAT_TachometerClass ESAT_Tachometer;
