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

#include "ESAT_FollowMagnetometerRunMode.h"
#include "ESAT_AttitudePIDController.h"

void ESAT_FollowMagnetometerRunModeClass::loop(const ESAT_AttitudeStateVector attitudeStateVector)
{
  ESAT_AttitudePIDController.loop(attitudeStateVector.magneticAngle,
                                  attitudeStateVector);
}

ESAT_FollowMagnetometerRunModeClass ESAT_FollowMagnetometerRunMode;
