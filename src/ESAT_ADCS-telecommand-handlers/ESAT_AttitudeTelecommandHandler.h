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

#ifndef ESAT_AttitudeTelecommandHandler_h
#define ESAT_AttitudeTelecommandHandler_h

#include <Arduino.h>
#include "ESAT_ADCSTelecommandHandler.h"

// Telecommand handler for attitude control telecommands.
// Use the global instance ESAT_AttitudeTelecommandHandler.
class ESAT_AttitudeTelecommandHandlerClass: public ESAT_ADCSTelecommandHandler
{
  public:
    // Either handle the telecommand provided by the packet and return
    // true, or do nothing and return false.
    boolean handleTelecommand(ESAT_CCSDSPacket telecommand);

  private:
    // Available telecommand codes.
    enum TelecommandCodes
    {
      FOLLOW_MAGNETIC_ANGLE = 0x00,
      FOLLOW_SUN_ANGLE = 0x01,
      ATTITUDE_CONTROLLER_SET_PROPORTIONAL_GAIN = 0x10,
      ATTITUDE_CONTROLLER_SET_INTEGRAL_GAIN = 0x11,
      ATTITUDE_CONTROLLER_SET_DERIVATIVE_GAIN = 0x12,
      ATTITUDE_CONTROLLER_RESET_ERROR_INTEGRAL = 0x13,
      ATTITUDE_CONTROLLER_USE_GYROSCOPE = 0x14,
      ATTITUDE_CONTROLLER_SET_ACTUATOR = 0x15,
      ATTITUDE_CONTROLLER_SET_DEADBAND = 0x16,
      ATTITUDE_CONTROLLER_SET_DETUMBLING_THRESHOLD = 0x17,
    };

    // Handle the telecommand for following the magnetic angle.
    // This changes the ADCS run mode.
    void handleFollowMagneticAngleTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for following the sun angle.
    // This changes the ADCS run mode.
    void handleFollowSunAngleTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the proportional gain of the
    // attitude controller.
    void handleAttitudeControllerSetProportionalGainTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the integral gain of the
    // attitude controller.
    void handleAttitudeControllerSetIntegralGainTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the derivative gain of the
    // attitude controller.
    void handleAttitudeControllerSetDerivativeGainTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for resetting the error integral of the
    // attitude controller.
    void handleAttitudeControllerResetErrorIntegralTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the usage of the gyroscope
    // for attitude control.
    void handleAttitudeControllerUseGyroscopeTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the actuator used for
    // attitude control.
    void handleAttitudeControllerSetActuatorTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the deadband of the attitude
    // controller.
    void handleAttitudeControllerSetDeadbandTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the detumbling threshold of
    // the attitude controller.
    void handleAttitudeControllerSetDetumblingThresholdTelecommand(ESAT_CCSDSPacket telecommand);
};

// Global instance of the attitude control telecommand handler library.
extern ESAT_AttitudeTelecommandHandlerClass ESAT_AttitudeTelecommandHandler;

#endif /* ESAT_AttitudeTelecommandHandler_h */
