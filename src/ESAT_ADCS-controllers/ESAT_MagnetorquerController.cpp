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

#include "ESAT_ADCS-controllers/ESAT_MagnetorquerController.h"
#include "ESAT_ADCS.h"

ESAT_MagnetorquerClass::Polarity ESAT_MagnetorquerControllerClass::clockwiseXPolarity()
{
  const ESAT_AttitudeStateVector attitudeStateVector =
    ESAT_ADCS.attitudeStateVector();
  const word magneticAngle = attitudeStateVector.magneticAngle;
  if ((magneticAngle % 360) < 90)
  {
    return ESAT_Magnetorquer.NEGATIVE;
  }
  if ((magneticAngle % 360) < 180)
  {
    return ESAT_Magnetorquer.NEGATIVE;
  }
  if ((magneticAngle % 360) < 270)
  {
    return ESAT_Magnetorquer.POSITIVE;
  }
  return ESAT_Magnetorquer.POSITIVE;
}

ESAT_MagnetorquerClass::Polarity ESAT_MagnetorquerControllerClass::clockwiseYPolarity()
{
  const ESAT_AttitudeStateVector attitudeStateVector =
    ESAT_ADCS.attitudeStateVector();
  const word magneticAngle = attitudeStateVector.magneticAngle;
  if ((magneticAngle % 360) < 90)
  {
    return ESAT_Magnetorquer.POSITIVE;
  }
  if ((magneticAngle % 360) < 180)
  {
    return ESAT_Magnetorquer.NEGATIVE;
  }
  if ((magneticAngle % 360) < 270)
  {
    return ESAT_Magnetorquer.NEGATIVE;
  }
  return ESAT_Magnetorquer.POSITIVE;
}

ESAT_MagnetorquerClass::Polarity ESAT_MagnetorquerControllerClass::counterclockwiseXPolarity()
{
  const ESAT_AttitudeStateVector attitudeStateVector =
    ESAT_ADCS.attitudeStateVector();
  const word magneticAngle = attitudeStateVector.magneticAngle;
  if ((magneticAngle % 360) < 90)
  {
    return ESAT_Magnetorquer.POSITIVE;
  }
  if ((magneticAngle % 360) < 180)
  {
    return ESAT_Magnetorquer.POSITIVE;
  }
  if ((magneticAngle % 360) < 270)
  {
    return ESAT_Magnetorquer.NEGATIVE;
  }
  return ESAT_Magnetorquer.NEGATIVE;
}

ESAT_MagnetorquerClass::Polarity ESAT_MagnetorquerControllerClass::counterclockwiseYPolarity()
{
  const ESAT_AttitudeStateVector attitudeStateVector =
    ESAT_ADCS.attitudeStateVector();
  const word magneticAngle = attitudeStateVector.magneticAngle;
  if ((magneticAngle % 360) < 90)
  {
    return ESAT_Magnetorquer.NEGATIVE;
  }
  if ((magneticAngle % 360) < 180)
  {
    return ESAT_Magnetorquer.POSITIVE;
  }
  if ((magneticAngle % 360) < 270)
  {
    return ESAT_Magnetorquer.POSITIVE;
  }
  return ESAT_Magnetorquer.NEGATIVE;
}

void ESAT_MagnetorquerControllerClass::rotateClockwise()
{
  ESAT_Magnetorquer.writeEnable(false);
  ESAT_Magnetorquer.writeX(clockwiseXPolarity());
  ESAT_Magnetorquer.writeY(clockwiseYPolarity());
  ESAT_Magnetorquer.writeEnable(true);
}

void ESAT_MagnetorquerControllerClass::rotateCounterclockwise()
{
  ESAT_Magnetorquer.writeEnable(false);
  ESAT_Magnetorquer.writeX(counterclockwiseXPolarity());
  ESAT_Magnetorquer.writeY(counterclockwiseYPolarity());
  ESAT_Magnetorquer.writeEnable(true);
}

void ESAT_MagnetorquerControllerClass::stop()
{
  ESAT_Magnetorquer.writeEnable(false);
}

ESAT_MagnetorquerControllerClass ESAT_MagnetorquerController;
