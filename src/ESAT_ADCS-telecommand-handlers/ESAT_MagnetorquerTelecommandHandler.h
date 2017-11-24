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

#ifndef ESAT_MagnetorquerTelecommandHandler_h
#define ESAT_MagnetorquerTelecommandHandler_h

#include <Arduino.h>
#include "ESAT_ADCSTelecommandHandler.h"

// Telecommand handler for magnetorquer control telecommands.
// Use the global instance ESAT_MagnetorquerTelecommandHandler.
class ESAT_MagnetorquerTelecommandHandlerClass: public ESAT_ADCSTelecommandHandler
{
  public:
    // Either handle the telecommand provided by the packet and return
    // true, or do nothing and return false.
    boolean handleTelecommand(ESAT_CCSDSPacket telecommand);

  private:
    // Available telecommand codes.
    enum TelecommandCodes
    {
      MAGNETORQUER_ENABLE = 0x30,
      MAGNETORQUER_SET_X_POLARITY = 0x31,
      MAGNETORQUER_SET_Y_POLARITY = 0x32,
      MAGNETORQUER_APPLY_MAXIMUM_TORQUE = 0x33,
      MAGNETORQUER_DEMAGNETIZE = 0x34,
    };

    // Handle the telecommand for enabling or disabling the magnetorquers.
    // The telecommand has one 8-bit boolean parameter:
    // true for enabling the the magnetorquer driver and false for
    // disabling it.
    // This changes the ADCS run mode to MAGNETORQUER_ENABLE.
    void handleMagnetorquerEnableTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the polarity of the X-axis
    // magnetorquer.
    // The telecommand has one 8-bit unsigned integer parameter:
    // 0 for negative polarity and 1 for positive polarity.
    // This changes the ADCS run mode to MAGNETORQUER_SET_X_POLARITY.
    void handleMagnetorquerSetXPolarityTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for setting the polarity of the Y-axis
    // magnetorquer.
    // The telecommand has one 8-bit unsigned integer parameter:
    // 0 for negative polarity and 1 for positive polarity.
    // This changes the ADCS run mode to MAGNETORQUR_SET_Y_POLARITY.
    void handleMagnetorquerSetYPolarityTelecommand(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for applying a maximum torque.
    // The telecommand has one 8-bit unsigned integer parameter:
    // 0 for clockwise rotation and 1 for counterclockwise rotation.
    // This changes the ADCS run mode to MAGNETORQUER_APPLY_MAXIMUM_TORQUE.
    void handleMagnetorquerApplyMaximumTorqueTelecommand(ESAT_CCSDSPacket telecommand);

    // HAndle the telecommand for demagnetizing the magnetorquer.
    // The telecommand has one 8-bit unsigned integer parameter:
    // the number of demagnetization cycles.
    // This changes the ADCS run mode to MAGNETORQUER_DEMAGNETIZE.
    void handleMagnetorquerDemagnetizeTelecommand(ESAT_CCSDSPacket telecommand);
};

// Global instance of the magnetorquer telecommand handler library.
extern ESAT_MagnetorquerTelecommandHandlerClass ESAT_MagnetorquerTelecommandHandler;

#endif /* ESAT_MagnetorquerTelecommandHandler_h */
