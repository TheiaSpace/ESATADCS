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

#ifndef ESAT_MagnetorquerSetXPolarityRunMode_h
#define ESAT_MagnetorquerSetXPolarityRunMode_h

#include <Arduino.h>
#include "ESAT_ADCSRunMode.h"
#include "ESAT_Magnetorquer.h"

// X-axis magnetorquer polarity setting run mode.
// Use the public instance ESAT_MagnetorquerSetXPolarityRunMode.
class ESAT_MagnetorquerSetXPolarityRunModeClass: public ESAT_ADCSRunMode
{
  public:
    // Desired polarity of the magnetorquer.
    ESAT_MagnetorquerClass::Polarity polarity;

    // Perform one iteration of setting the polarity of the
    // magnetorquer.
    void loop(ESAT_AttitudeStateVector attitudeStateVector);
};

// Global instance of the X-axis magnetorquer polarity setting run
// mode library.
extern ESAT_MagnetorquerSetXPolarityRunModeClass ESAT_MagnetorquerSetXPolarityRunMode;

#endif /* ESAT_MagnetorquerSetXPolarityRunMode_h */
