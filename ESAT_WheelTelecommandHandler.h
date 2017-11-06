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

#ifndef ESAT_WheelTelecommandHandler_h
#define ESAT_WheelTelecommandHandler_h

#include <Arduino.h>
#include "ESAT_ADCSTelecommandHandler.h"

// Telecommand handler for wheel control telecommands.
// Use the global instance ESAT_WheelTelecommandHandler.
class ESAT_WheelTelecommandHandlerClass: public ESAT_ADCSTelecommandHandler
{
  public:
    // Either handle the telecommand provided by the packet and return
    // true, or do nothing and return false.
    boolean handleTelecommand(ESAT_CCSDSPacket telecommand);

  private:
    // Available telecommand codes.
    enum TelecommandCodes
    {
      WHEEL_SET_DUTY_CYCLE = 0x20,
      WHEEL_SET_SPEED = 0x21,
      WHEEL_CONTROLLER_SET_PROPORTIONAL_GAIN = 0x22,
      WHEEL_CONTROLLER_SET_INTEGRAL_GAIN = 0x23,
      WHEEL_CONTROLLER_SET_DERIVATIVE_GAIN = 0x24,
      WHEEL_CONTROLLER_RESET_ERROR_INTEGRAL = 0x25,
    };

    // Handle the telecommand for setting the duty cycle of the wheel.
    // This changes the ADCS run mode.
    void handleWheelSetDutyCycleTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the wheel speed.
    // This changes the ADCS run mode.
    void handleWheelSetSpeedTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the proportional gain of the
    // wheel controller.
    void handleWheelControllerSetProportionalGainTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the integral gain of the
    // wheel controller.
    void handleWheelControllerSetIntegralGainTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the derivative gain of the
    // wheel controller.
    void handleWheelControllerSetDerivativeGainTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for resetting the error integral of the
    // wheel controller.
    void handleWheelControllerResetErrorIntegralTelecommand(ESAT_CCSDSPacket telecommand);
};

// Global instance of the wheel telecommand handler library.
extern ESAT_WheelTelecommandHandlerClass ESAT_WheelTelecommandHandler;

#endif /* ESAT_WheelTelecommandHandler_h */
