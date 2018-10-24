/*
 * Copyright (C) 2017, 2018 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_ADCSTelecommandHandler_h
#define ESAT_ADCSTelecommandHandler_h

#include <Arduino.h>
#include <ESAT_CCSDSPacket.h>

// An ADCS telecommand handler has a handleTelecommand() function that
// accepts a telecommand packet and, if possible, performs the
// requested actions.  The user can register ADCS telecommand handlers
// with ESAT_ADCS.registerTelecommandHandler() to add new telecommands
// to the ADCS.
class ESAT_ADCSTelecommandHandler
{
  public:
    // Next telecommand handler in the list of registered telecommand
    // handlers.
    // ESAT_ADCS uses this to keep a linked list of registered
    // telecommand handlers: it can traverse the list by going from
    // one telecommand handler to the next telecommand handler until
    // reaching the end of the list at nullptr.
    // Only ESAT_ADCS should care about this.
    ESAT_ADCSTelecommandHandler* nextTelecommandHandler = nullptr;

    virtual ~ESAT_ADCSTelecommandHandler() {};

    // Either handle the telecommand provided by the packet and return
    // true, or do nothing and return false.
    virtual boolean handleTelecommand(ESAT_CCSDSPacket telecommand) = 0;
};

#endif /* ESAT_ADCSTelecommandHandler_h */
