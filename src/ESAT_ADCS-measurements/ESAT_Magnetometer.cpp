/*
 * Copyright (C) 2017, 2018 Theia Space, Universidad Politécnica de Madrid
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

#include "ESAT_ADCS-measurements/ESAT_Magnetometer.h"
#include <ESAT_Util.h>

void ESAT_MagnetometerClass::begin()
{
  error = false;
#ifdef ARDUINO_ESAT_ADCS
  bus = &WireADCS;
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  bus = &WireOBC;
#endif /* ARDUINO_ESAT_OBC */
  setBypassMode();
}

word ESAT_MagnetometerClass::computeAttitude(const float xField,
                                             const float yField) const
{
  // To estimate the attitude, it is necessary to know the angle of
  // the magnetic field, which can be deduced from the measured
  // components of the magnetic field and a little knowledge of the
  // orientation of the magnetometer.
  // The field angle is expressed in radians.
#ifdef ARDUINO_ESAT_OBC
  const float fieldAngle = atan2(xField, yField);
#endif /* ARDUINO_ESAT_OBC */
#ifdef ARDUINO_ESAT_ADCS
  const float fieldAngle = -atan2(xField, yField);
#endif /* ARDUINO_ESAT_ADCS */
  // The following equation approximates the relationship between the
  // attitude of the satellite and the measured field angle:
  // fieldAngle(attitude) = attitude
  //                      + COSINE_CALIBRATION_COEFFICIENT * cos(2 * attitude)
  //                      + SINE_CALIBRATION_COEFFICIENT * sin(2 * attitude).
  // In the above equation, all angles are expressed in radians.
  // In general, COSINE_CALIBRATION_COEFFICIENT and SINE_CALIBRATION_COEFFICIENT
  // are non-zero because the magnetometer isn't at the geometric centre of the
  // satellite, so it is necessary to solve the following implicit equation
  // for the attitude:
  // function(attitude) = fieldAngle - fieldAngle(attitude) = 0.
  // The following lines solve the equation approximately using Newton's method,
  // which gives an improved guess attitude_n+1 from a guess attitude_n through
  // the expression
  // attitude_n+1 = attitude_n - function(attitude_n) / derivative(attitude_n),
  // where derivative(attitude) = 1 - d[fieldAngle(attitude)]/d[attitude].
  float guess = fieldAngle;
  for (int iteration = 0;
       iteration < ATTITUDE_COMPUTATION_ITERATIONS;
       iteration = iteration + 1)
  {
    const float function =
      fieldAngle
      - guess
      - COSINE_CALIBRATION_COEFFICIENT * cos(2 * guess)
      - SINE_CALIBRATION_COEFFICIENT * sin(2 * guess);
    const float derivative =
      -1
      + 2 * COSINE_CALIBRATION_COEFFICIENT * sin(2 * guess)
      - 2 * SINE_CALIBRATION_COEFFICIENT * cos(2 * guess);
    guess = guess - function / derivative;
  }
  // The computed attitude must be expressed in degrees; as the guess
  // is expressed in radians, it must be converted to degrees.
  const int attitude = round(guess * RAD_TO_DEG);
  // In addition, the attitude angle must range from 0 degrees to
  // 359 degrees.
  return normaliseAttitude(attitude);
}

word ESAT_MagnetometerClass::getReading()
{
  bus->beginTransmission(MAGNETOMETER_ADDRESS);
  bus->write(READING_REGISTER);
  const byte writeStatus = bus->endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = bus->requestFrom(int(MAGNETOMETER_ADDRESS), 4);
  if (bytesRead != 4)
  {
    error = true;
    return 0;
  }
  const byte xLowByte = bus->read();
  const byte xHighByte = bus->read();
  const byte yLowByte = bus->read();
  const byte yHighByte = bus->read();
  const word xFieldBits = word(xHighByte, xLowByte);
  const word yFieldBits = word(yHighByte, yLowByte);
  const int xField = ESAT_Util.wordToInt(xFieldBits);
  const int yField = ESAT_Util.wordToInt(yFieldBits);
  return computeAttitude(xField, yField);
}

word ESAT_MagnetometerClass::normaliseAttitude(const int attitude) const
{
  if (attitude < 0)
  {
    return attitude + 360;
  }
  else if (attitude >= 360)
  {
    return attitude - 360;
  }
  else
  {
    return attitude;
  }
}

word ESAT_MagnetometerClass::read()
{
  setBypassMode();
  startReading();
  waitForReading();
  return getReading();
}

void ESAT_MagnetometerClass::setBypassMode()
{
  bus->beginTransmission(CHIP_ADDRESS);
  bus->write(BYPASS_REGISTER);
  bus->write(ENABLE_BYPASS);
  const byte writeStatus = bus->endTransmission();
  if (writeStatus != 0)
  {
    error = true;
  }
}

void ESAT_MagnetometerClass::startReading()
{
  bus->beginTransmission(MAGNETOMETER_ADDRESS);
  bus->write(CONTROL_REGISTER);
  bus->write(SINGLE_MEASUREMENT_MODE);
  const byte writeStatus = bus->endTransmission();
  if (writeStatus != 0)
  {
    error = true;
  }
}

void ESAT_MagnetometerClass::waitForReading()
{
  const byte timeout = 255;
  for (int i = 0; i < timeout; i++)
  {
    bus->beginTransmission(MAGNETOMETER_ADDRESS);
    bus->write(DATA_STATUS_REGISTER);
    const byte writeStatus = bus->endTransmission();
    if (writeStatus != 0)
    {
      error = true;
      return;
    }
    const byte bytesRead = bus->requestFrom(int(MAGNETOMETER_ADDRESS), 1);
    if (bytesRead != 1)
    {
      error = true;
      return;
    }
    const byte readingState = bus->read();
    if ((readingState & DATA_READY) != 0)
    {
      return;
    }
  }
  error = true;
}

ESAT_MagnetometerClass ESAT_Magnetometer;
