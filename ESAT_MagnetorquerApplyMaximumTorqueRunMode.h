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

#ifndef ESAT_MagnetorquerApplyMaximumTorqueRunMode_h
#define ESAT_MagnetorquerApplyMaximumTorqueRunMode_h

#include <Arduino.h>
#include "ESAT_ADCSRunMode.h"
#include "ESAT_Magnetorquer.h"

// Magnetorquer maximum torque mode.
// Use the public instance ESAT_MagnetorquerApplyMaximumTorqueRunMode.
class ESAT_MagnetorquerApplyMaximumTorqueRunModeClass: public ESAT_ADCSRunMode
{
  public:
    // Available modes of operation.
    enum Mode
    {
      ROTATE_CLOCKWISE,
      ROTATE_COUNTERCLOCKWISE,
      STOP,
    };

    // Current mode of operation.
    Mode mode;

    // Perform one iteration of the maximum magnetic torque mode:
    // apply the maximum possible torque towards the specified direction
    // given the attitude of the satellite.
    void loop(ESAT_AttitudeStateVector attitudeStateVector);

  private:
    // Return the X-axis magnetorquer polarity needed for clockwise
    // rotation at the given magnetic angle (the angle in degrees of
    // the magnetic North with respect to the +X axis of the
    // satellite, positive counterclockwise).
    ESAT_MagnetorquerClass::Polarity clockwiseXPolarity(word magneticAngle);

    // Return the Y-axis magnetorquer polarity needed for clockwise
    // rotation at the given magnetic angle (the angle in degrees of
    // the magnetic North with respect to the +X axis of the
    // satellite, positive counterclockwise).
    ESAT_MagnetorquerClass::Polarity clockwiseYPolarity(word magneticAngle);

    // Return the X-axis magnetorquer polarity needed for
    // counterclockwise rotation at the given magnetic angle (the
    // angle in degrees of the magnetic North with respect to the +X
    // axis of the satellite, positive counterclockwise).
    ESAT_MagnetorquerClass::Polarity counterclockwiseXPolarity(word magneticAngle);

    // Return the Y-axis magnetorquer polarity needed for
    // counterclockwise rotation at the given magnetic angle (the
    // angle in degrees of the magnetic North with respect to the +X
    // axis of the satellite, positive counterclockwise).
    ESAT_MagnetorquerClass::Polarity counterclockwiseYPolarity(word magneticAngle);

    // Use the magnetorquers to rotate the satellite in the clockwise
    // direction given the magnetic angle (the angle in degrees of the
    // magnetic North with respect to the +X axis of the satellite,
    // positive counterclockwise).
    void rotateClockwise(word magneticAngle);

    // Use the magnetorquers to rotate the satellite in the
    // counterclockwise direction given the magnetic angle (the angle
    // in degrees of the magnetic North with respect to the +X axis of
    // the satellite, positive counterclockwise).
    void rotateCounterclockwise(word magneticAngle);

    // Stop using the magnetorquers.
    void stop();
};

// Global instance of the magnetorquer maximum torque run mode library.
extern ESAT_MagnetorquerApplyMaximumTorqueRunModeClass ESAT_MagnetorquerApplyMaximumTorqueRunMode;

#endif /* ESAT_MagnetorquerApplyMaximumTorquerRunMode_h */
