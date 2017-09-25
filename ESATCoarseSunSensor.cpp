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

#include "ESATCoarseSunSensor.h"

void ESATCoarseSunSensor::begin()
{
  pinMode(pinXPlus, INPUT);
  pinMode(pinXMinus, INPUT);
  pinMode(pinYPlus, INPUT);
  pinMode(pinYMinus, INPUT);
}

int ESATCoarseSunSensor::read()
{
  const int readingXPlus = analogRead(pinXPlus);
  const int readingXMinus = analogRead(pinXMinus);
  const int readingYPlus = analogRead(pinYPlus);
  const int readingYMinus = analogRead(pinYMinus);
  const int nx = readingXPlus - readingXMinus;
  const int ny = readingYPlus - readingYMinus;
  const int reading = round(atan2(ny, nx) * RAD_TO_DEG) % 360;
  return reading;
}

ESATCoarseSunSensor CoarseSunSensor;
