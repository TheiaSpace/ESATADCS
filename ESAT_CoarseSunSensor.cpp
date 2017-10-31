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

#include "ESAT_CoarseSunSensor.h"

void ESAT_CoarseSunSensorClass::begin()
{
  pinMode(PIN_X_PLUS, INPUT);
  pinMode(PIN_X_MINUS, INPUT);
  pinMode(PIN_Y_PLUS, INPUT);
  pinMode(PIN_Y_MINUS, INPUT);
}

int ESAT_CoarseSunSensorClass::read()
{
  const int readingXPlus = analogRead(PIN_X_PLUS);
  const int readingXMinus = analogRead(PIN_X_MINUS);
  const int readingYPlus = analogRead(PIN_Y_PLUS);
  const int readingYMinus = analogRead(PIN_Y_MINUS);
  const int nx = readingXPlus - readingXMinus;
  const int ny = readingYPlus - readingYMinus;
  const int reading = round(atan2(ny, nx) * RAD_TO_DEG);
  if (reading < 0)
  {
    return reading + 360;
  }
  else
  {
    return reading;
  }
}

ESAT_CoarseSunSensorClass ESAT_CoarseSunSensor;
