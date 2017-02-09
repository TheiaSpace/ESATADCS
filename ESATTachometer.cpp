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

#include "ESATTachometer.h"

void ESATTachometer::begin()
{
  count = 0;
  previousCount = 0;
  previousReadingTime = 0;
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(pin, incrementCounter, FALLING);
}

void ESATTachometer::incrementCounter()
{
  Tachometer.count = Tachometer.count + 1;
}

unsigned int ESATTachometer::read()
{
  const unsigned long currentTime = millis();
  const unsigned long ellapsedMilliseconds = currentTime - previousReadingTime;
  previousReadingTime = currentTime;
  const unsigned long millisecondsPerMinute = 60000;
  const unsigned int reading = (long(millisecondsPerMinute / countsPerRevolution)
          * long(count - previousCount)) / ellapsedMilliseconds;
  previousCount = count;
  return reading;
}

ESATTachometer Tachometer;
