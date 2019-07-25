/*
 * Copyright (C) 2017, 2018, 2019 Theia Space, Universidad Politécnica de Madrid
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
#ifdef ARDUINO_ESAT_ADCS
#include <EEPROM.h>
#endif /* ARDUINO_ESAT_ADCS */
#include <ESAT_Util.h>
#ifdef ARDUINO_ESAT_OBC
#include <SD.h>
#endif /* ARDUINO_ESAT_OBC */

#ifdef ARDUINO_ESAT_OBC
const char ESAT_MagnetometerClass::GEOMETRY_FILENAME[] = "maggeom";
#endif /* ARDUINO_ESAT_OBC */

float ESAT_MagnetometerClass::angleDifference(const float minuend,
                                              const float subtrahend) const
{
  const float difference = minuend - subtrahend;
  if (difference > 180)
  {
    return difference - 360;
  }
  if (difference < -180)
  {
    return difference + 360;
  }
  return difference;
}

void ESAT_MagnetometerClass::begin()
{
  error = false;
  readGeometryCorrection();
  setBypassMode();
}

word ESAT_MagnetometerClass::computeAttitude(const float xField,
                                             const float yField) const
{
  // To estimate the attitude, it is necessary to know the angle of
  // the magnetic field, which can be deduced from the measured
  // components of the magnetic field and a little knowledge of the
  // orientation of the magnetometer.
#ifdef ARDUINO_ESAT_OBC
  const float fieldAngle =
    normaliseAttitude(atan2(xField, yField) * RAD_TO_DEG);
#endif /* ARDUINO_ESAT_OBC */
#ifdef ARDUINO_ESAT_ADCS
  const float fieldAngle =
    normaliseAttitude(-atan2(xField, yField) * RAD_TO_DEG);
#endif /* ARDUINO_ESAT_ADCS */
  // The attitude is obtained from a piecewise linear interpolation of the
  // measured angles.
  const int actualAngles[] = {315, 0, 45, 90, 135, 180, 225, 270, 315, 0};
  for (int position = 0;
       position < (GEOMETRY_CORRECTION_POSITIONS + 1);
       position = position + 1)
  {
    if ((angleDifference(fieldAngle, fieldAngles[position]) >= 0)
        && (angleDifference(fieldAngle, fieldAngles[position + 1]) <= 0))
    {
      const float attitude =
        fieldAngle
        + angleDifference(actualAngles[position], fieldAngles[position])
        * angleDifference(fieldAngle, fieldAngles[position + 1])
        / angleDifference(fieldAngles[position], fieldAngles[position + 1])
        + angleDifference(actualAngles[position + 1], fieldAngles[position + 1])
        * angleDifference(fieldAngle, fieldAngles[position])
        / angleDifference(fieldAngles[position + 1], fieldAngles[position]);
      return normaliseAttitude(attitude);
    }
  }
  return fieldAngle;
}

void ESAT_MagnetometerClass::configureGeometryCorrection(const word measurement0,
                                                         const word measurement45,
                                                         const word measurement90,
                                                         const word measurement135,
                                                         const word measurement180,
                                                         const word measurement225,
                                                         const word measurement270,
                                                         const word measurement315)
{
  // The geometry correction algorithm uses piecewise linear interpolation
  // with a periodic boundary condition.  The expressions are simpler with
  // two padding angles, one at each boundary.
  fieldAngles[0] = measurement315;
  fieldAngles[1] = measurement0;
  fieldAngles[2] = measurement45;
  fieldAngles[3] = measurement90;
  fieldAngles[4] = measurement135;
  fieldAngles[5] = measurement180;
  fieldAngles[6] = measurement225;
  fieldAngles[7] = measurement270;
  fieldAngles[8] = measurement315;
  fieldAngles[9] = measurement0;
  writeGeometryCorrection();
}

word ESAT_MagnetometerClass::getReading()
{
  bus.beginTransmission(MAGNETOMETER_ADDRESS);
  bus.write(READING_REGISTER);
  const byte writeStatus = bus.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = bus.requestFrom(int(MAGNETOMETER_ADDRESS), 4);
  if (bytesRead != 4)
  {
    error = true;
    return 0;
  }
  const byte xLowByte = bus.read();
  const byte xHighByte = bus.read();
  const byte yLowByte = bus.read();
  const byte yHighByte = bus.read();
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

void ESAT_MagnetometerClass::readGeometryCorrection()
{
#ifdef ARDUINO_ESAT_ADCS
  for (int position = 0;
       position < GEOMETRY_CORRECTION_POSITIONS;
       position = position + 1)
  {
    const byte highByte =
      EEPROM.read(GEOMETRY_EEPROM_ADDRESS + 2 * position);
    const byte lowByte =
      EEPROM.read(GEOMETRY_EEPROM_ADDRESS + 2 * position + 1);
    fieldAngles[position + 1] = word(highByte, lowByte);
  }
  fieldAngles[0] = fieldAngles[GEOMETRY_CORRECTION_POSITIONS];
  fieldAngles[GEOMETRY_CORRECTION_POSITIONS + 1] = fieldAngles[1];
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  File file = SD.open(GEOMETRY_FILENAME, FILE_READ);
  if (file.available() == GEOMETRY_CORRECTION_POSITIONS * 4)
  {
    for (int position = 0;
         position < GEOMETRY_CORRECTION_POSITIONS;
         position = position + 1)
    {
      const byte highByte = file.read();
      const byte lowByte = file.read();
      fieldAngles[position + 1] = word(highByte, lowByte);
    }
    fieldAngles[0] = fieldAngles[GEOMETRY_CORRECTION_POSITIONS];
    fieldAngles[GEOMETRY_CORRECTION_POSITIONS + 1] = fieldAngles[1];
  }
  else
  {
    fieldAngles[0] = 315;
    fieldAngles[1] = 0;
    fieldAngles[2] = 45;
    fieldAngles[3] = 90;
    fieldAngles[4] = 135;
    fieldAngles[5] = 180;
    fieldAngles[6] = 225;
    fieldAngles[7] = 270;
    fieldAngles[8] = 315;
    fieldAngles[9] = 0;
  }
  file.close();
#endif /* ARDUINO_ESAT_OBC */
}

void ESAT_MagnetometerClass::setBypassMode()
{
  bus.beginTransmission(CHIP_ADDRESS);
  bus.write(BYPASS_REGISTER);
  bus.write(ENABLE_BYPASS);
  const byte writeStatus = bus.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
  }
}

void ESAT_MagnetometerClass::startReading()
{
  bus.beginTransmission(MAGNETOMETER_ADDRESS);
  bus.write(CONTROL_REGISTER);
  bus.write(SINGLE_MEASUREMENT_MODE);
  const byte writeStatus = bus.endTransmission();
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
    bus.beginTransmission(MAGNETOMETER_ADDRESS);
    bus.write(DATA_STATUS_REGISTER);
    const byte writeStatus = bus.endTransmission();
    if (writeStatus != 0)
    {
      error = true;
      return;
    }
    const byte bytesRead = bus.requestFrom(int(MAGNETOMETER_ADDRESS), 1);
    if (bytesRead != 1)
    {
      error = true;
      return;
    }
    const byte readingState = bus.read();
    if ((readingState & DATA_READY) != 0)
    {
      return;
    }
  }
  error = true;
}

void ESAT_MagnetometerClass::writeGeometryCorrection()
{
#ifdef ARDUINO_ESAT_ADCS
  for (int position = 0;
       position < GEOMETRY_CORRECTION_POSITIONS;
       position = position + 1)
  {
    (void) EEPROM.write(GEOMETRY_EEPROM_ADDRESS + 2 * position,
                        highByte(fieldAngles[position + 1]));
    (void) EEPROM.write(GEOMETRY_EEPROM_ADDRESS + 2 * position + 1,
                        lowByte(fieldAngles[position + 1]));
  }
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  File file = SD.open(GEOMETRY_FILENAME, FILE_WRITE);
  (void) file.seek(0);
  for (int position = 0;
       position < GEOMETRY_CORRECTION_POSITIONS;
       position = position + 1)
  {
    (void) file.write(fieldAngles[position + 1]);
    (void) file.write(fieldAngles[position + 1]);
  }
  file.close();
#endif /* ARDUINO_ESAT_OBC */
}

ESAT_MagnetometerClass ESAT_Magnetometer;
