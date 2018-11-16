/*
 * Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_ADCSClockTelecommandHandler_h
#define ESAT_ADCSClockTelecommandHandler_h

#include <Arduino.h>
#include "ESAT_ADCSTelecommandHandler.h"

// Telecommand handler for managing the clock.
// Use the global instance ESAT_ADCSClockTelecommandHandler.
class ESAT_ADCSClockTelecommandHandlerClass: public ESAT_ADCSTelecommandHandler
{
  public:
    // Either handle the telecommand provided by the packet and return
    // true, or do nothing and return false.
    boolean handleTelecommand(ESAT_CCSDSPacket telecommand);

  private:
    // Available telecommand codes.
    enum TelecommandCodes
    {
      SET_TIME = 0xC0,
    };

    // Handle the telecommand for setting the time.
    // The telecommand has a timestamp parameter.
    void handleSetTimeTelecommand(ESAT_CCSDSPacket telecommand);
};

// Global instance of the clock management telecommand handler library.
extern ESAT_ADCSClockTelecommandHandlerClass ESAT_ADCSClockTelecommandHandler;

#endif /* ESAT_ADCSClockTelecommandHandler_h */
