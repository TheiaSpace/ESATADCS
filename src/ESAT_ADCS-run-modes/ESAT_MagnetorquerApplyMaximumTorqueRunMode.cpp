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

#include "ESAT_ADCS-run-modes/ESAT_MagnetorquerApplyMaximumTorqueRunMode.h"
#include "ESAT_ADCS.h"
#include "ESAT_ADCS-controllers/ESAT_MagnetorquerController.h"

byte ESAT_MagnetorquerApplyMaximumTorqueRunModeClass::identifier()
{
  return 0x33;
}

void ESAT_MagnetorquerApplyMaximumTorqueRunModeClass::run()
{
  const ESAT_AttitudeStateVector attitudeStateVector =
    ESAT_ADCS.attitudeStateVector();
  switch (mode)
  {
    case ROTATE_CLOCKWISE:
      ESAT_MagnetorquerController.rotateClockwise(attitudeStateVector.magneticAngle);
      break;
    case ROTATE_COUNTERCLOCKWISE:
      ESAT_MagnetorquerController.rotateCounterclockwise(attitudeStateVector.magneticAngle);
      break;
    default:
      break;
  }
}

ESAT_MagnetorquerApplyMaximumTorqueRunModeClass ESAT_MagnetorquerApplyMaximumTorqueRunMode;
