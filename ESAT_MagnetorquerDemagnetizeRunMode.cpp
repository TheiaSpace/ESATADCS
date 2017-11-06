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

#include "ESAT_MagnetorquerDemagnetizeRunMode.h"
#include "ESAT_Magnetorquer.h"

byte ESAT_MagnetorquerDemagnetizeRunModeClass::identifier()
{
  return 0x34;
}

void ESAT_MagnetorquerDemagnetizeRunModeClass::loop(ESAT_AttitudeStateVector attitudeStateVector)
{
  if (cycles == 0)
  {
    return;
  }
  ESAT_Magnetorquer.writeEnable(true);
  for (int i = 0; i < cycles; i++)
  {
    ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.POSITIVE);
    ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.POSITIVE);
    delay(20);
    ESAT_Magnetorquer.writeX(ESAT_Magnetorquer.NEGATIVE);
    ESAT_Magnetorquer.writeY(ESAT_Magnetorquer.NEGATIVE);
    delay(20);
  }
  ESAT_Magnetorquer.writeEnable(false);
  cycles = 0;
}

ESAT_MagnetorquerDemagnetizeRunModeClass ESAT_MagnetorquerDemagnetizeRunMode;
