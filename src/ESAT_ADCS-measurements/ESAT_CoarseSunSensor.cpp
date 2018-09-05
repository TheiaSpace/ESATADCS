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

#include "ESAT_ADCS-measurements/ESAT_CoarseSunSensor.h"

void ESAT_CoarseSunSensorClass::begin()
{
  pinMode(PIN_X_PLUS, INPUT);
  pinMode(PIN_X_MINUS, INPUT);
  pinMode(PIN_Y_PLUS, INPUT);
  pinMode(PIN_Y_MINUS, INPUT);
  setXPlusReadingsSource(READ_X_PLUS);
  setXMinusReadingsSource(READ_X_MINUS);
  setYPlusReadingsSource(READ_Y_PLUS);
  setYMinusReadingsSource(READ_Y_MINUS);
}

void ESAT_CoarseSunSensorClass::setXMinusReadingsSource(const ESAT_CoarseSunSensorClass::ReadingsSource source)
{
  sourceXMinus = source;
}

void ESAT_CoarseSunSensorClass::setXPlusReadingsSource(const ESAT_CoarseSunSensorClass::ReadingsSource source)
{
  sourceXPlus = source;
}

void ESAT_CoarseSunSensorClass::setYMinusReadingsSource(const ESAT_CoarseSunSensorClass::ReadingsSource source)
{
  sourceYMinus = source;
}

void ESAT_CoarseSunSensorClass::setYPlusReadingsSource(const ESAT_CoarseSunSensorClass::ReadingsSource source)
{
  sourceYPlus = source;
}

word ESAT_CoarseSunSensorClass::readSunAngle()
{
  const float readingXPlus = readXPlus();
  const float readingXMinus = readXMinus();
  const float readingYPlus = readYPlus();
  const float readingYMinus = readYMinus();
  const float nx = readingXPlus - readingXMinus;
  const float ny = readingYPlus - readingYMinus;
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

float ESAT_CoarseSunSensorClass::read(const ESAT_CoarseSunSensorClass::ReadingsSource source)
{
  switch (source)
  {
    case READ_X_PLUS:
      return analogRead(PIN_X_PLUS) * (100. / MEASUREMENT_RANGE);
      break;
    case READ_Y_PLUS:
      return analogRead(PIN_Y_PLUS) * (100. / MEASUREMENT_RANGE);
      break;
    case READ_X_MINUS:
      return analogRead(PIN_X_MINUS) * (100. / MEASUREMENT_RANGE);
      break;
    case READ_Y_MINUS:
      return analogRead(PIN_Y_MINUS) * (100. / MEASUREMENT_RANGE);
      break;
    case READ_DARK:
      return 0;
      break;
    case READ_SATURATED:
      return 100;
      break;
    default:
      return 0;
      break;
  }
  return 0;
}

float ESAT_CoarseSunSensorClass::readXMinus()
{
  return read(sourceXMinus);
}

float ESAT_CoarseSunSensorClass::readXPlus()
{
  return read(sourceXPlus);
}

float ESAT_CoarseSunSensorClass::readYMinus()
{
  return read(sourceYMinus);
}

float ESAT_CoarseSunSensorClass::readYPlus()
{
  return read(sourceYPlus);
}

ESAT_CoarseSunSensorClass ESAT_CoarseSunSensor;
