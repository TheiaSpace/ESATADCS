/*
 * Copyright (C) 2017 Theia Space, Universidad Politécnica de Madrid
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

#include "ESAT_ADCS-measurements/ESAT_Tachometer.h"

void ESAT_TachometerClass::begin()
{
  count = 0;
  previousCount = 0;
  previousReadingTime = 0;
  pinMode(PIN, INPUT_PULLUP);
  attachInterrupt(PIN, incrementCounter, FALLING);
}

void ESAT_TachometerClass::incrementCounter()
{
  ESAT_Tachometer.count = ESAT_Tachometer.count + 1;
}

unsigned int ESAT_TachometerClass::read()
{
  const unsigned long currentTime = millis();
  const unsigned long ellapsedMilliseconds = currentTime - previousReadingTime;
  previousReadingTime = currentTime;
  const unsigned long millisecondsPerMinute = 60000;
  const unsigned int reading =
    (long(millisecondsPerMinute / COUNTS_PER_REVOLUTION)
     * long(count - previousCount)) / ellapsedMilliseconds;
  previousCount = count;
  return reading;
}

ESAT_TachometerClass ESAT_Tachometer;
