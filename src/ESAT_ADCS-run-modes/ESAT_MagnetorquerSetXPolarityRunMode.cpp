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

#include "ESAT_ADCS-run-modes/ESAT_MagnetorquerSetXPolarityRunMode.h"

byte ESAT_MagnetorquerSetXPolarityRunModeClass::identifier()
{
  return 0x31;
}

void ESAT_MagnetorquerSetXPolarityRunModeClass::run()
{
  ESAT_Magnetorquer.writeX(polarity);
}

ESAT_MagnetorquerSetXPolarityRunModeClass ESAT_MagnetorquerSetXPolarityRunMode;
