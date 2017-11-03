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

#include "ESAT_MagnetorquerApplyMaximumTorqueRunMode.h"

ESAT_MagnetorquerClass::Polarity ESAT_MagnetorquerApplyMaximumTorqueRunModeClass::clockwiseXPolarity(const word magneticAngle)
{
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

ESAT_MagnetorquerClass::Polarity ESAT_MagnetorquerApplyMaximumTorqueRunModeClass::clockwiseYPolarity(const word magneticAngle)
{
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

ESAT_MagnetorquerClass::Polarity ESAT_MagnetorquerApplyMaximumTorqueRunModeClass::counterclockwiseXPolarity(const word magneticAngle)
{
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

ESAT_MagnetorquerClass::Polarity ESAT_MagnetorquerApplyMaximumTorqueRunModeClass::counterclockwiseYPolarity(const word magneticAngle)
{
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

void ESAT_MagnetorquerApplyMaximumTorqueRunModeClass::loop(ESAT_AttitudeStateVector attitudeStateVector)
{
  switch (mode)
  {
    case STOP:
      stop();
      break;
    case ROTATE_CLOCKWISE:
      rotateClockwise(attitudeStateVector.magneticAngle);
      break;
    case ROTATE_COUNTERCLOCKWISE:
      rotateCounterclockwise(attitudeStateVector.magneticAngle);
      break;
    default:
      break;
  }
}

void ESAT_MagnetorquerApplyMaximumTorqueRunModeClass::rotateClockwise(const word magneticAngle)
{
  ESAT_Magnetorquer.writeEnable(false);
  ESAT_Magnetorquer.writeX(clockwiseXPolarity(magneticAngle));
  ESAT_Magnetorquer.writeY(clockwiseYPolarity(magneticAngle));
  ESAT_Magnetorquer.writeEnable(true);
}

void ESAT_MagnetorquerApplyMaximumTorqueRunModeClass::rotateCounterclockwise(const word magneticAngle)
{
  ESAT_Magnetorquer.writeEnable(false);
  ESAT_Magnetorquer.writeX(counterclockwiseXPolarity(magneticAngle));
  ESAT_Magnetorquer.writeY(counterclockwiseYPolarity(magneticAngle));
  ESAT_Magnetorquer.writeEnable(true);
}

void ESAT_MagnetorquerApplyMaximumTorqueRunModeClass::stop()
{
  ESAT_Magnetorquer.writeEnable(false);
}

ESAT_MagnetorquerApplyMaximumTorqueRunModeClass ESAT_MagnetorquerApplyMaximumTorqueRunMode;
