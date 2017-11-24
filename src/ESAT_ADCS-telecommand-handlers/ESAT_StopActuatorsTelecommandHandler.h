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

#ifndef ESAT_StopActuatorsTelecommandHandler_h
#define ESAT_StopActuatorsTelecommandHandler_h

#include <Arduino.h>
#include "ESAT_ADCSTelecommandHandler.h"

// Telecommand handler for stopping the actuators.
// Use the global instance ESAT_StopActuatorsTelecommandHandler.
class ESAT_StopActuatorsTelecommandHandlerClass: public ESAT_ADCSTelecommandHandler
{
  public:
    // Either handle the telecommand provided by the packet and return
    // true, or do nothing and return false.
    boolean handleTelecommand(ESAT_CCSDSPacket telecommand);

  private:
    // Available telecommand codes.
    enum TelecommandCodes
    {
      STOP_ACTUATORS = 0xFF,
    };

    // HAndle the telecommand for stopping the actuators.
    // The telecommand has no parameters.
    // This changes the ADCS run mode to STOP_ACTUATORS.
    void handleStopActuatorsTelecommand(ESAT_CCSDSPacket telecommand);
};

// Global instance of the actuators stopping telecommand handler library.
extern ESAT_StopActuatorsTelecommandHandlerClass ESAT_StopActuatorsTelecommandHandler;

#endif /* ESAT_StopActuatorsTelecommandHandler_h */
