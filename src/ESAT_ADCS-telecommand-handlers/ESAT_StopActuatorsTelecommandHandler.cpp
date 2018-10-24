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

#include "ESAT_ADCS-telecommand-handlers/ESAT_StopActuatorsTelecommandHandler.h"
#include "ESAT_ADCS.h"
#include "ESAT_ADCS-run-modes/ESAT_StopActuatorsRunMode.h"

boolean ESAT_StopActuatorsTelecommandHandlerClass::handleTelecommand(ESAT_CCSDSPacket telecommand)
{
  telecommand.rewind();
  const ESAT_CCSDSSecondaryHeader secondaryHeader =
    telecommand.readSecondaryHeader();
  switch (secondaryHeader.packetIdentifier)
  {
    case STOP_ACTUATORS:
      handleStopActuatorsTelecommand(telecommand);
      return true;
      break;
    default:
      return false;
      break;
  }
}

void ESAT_StopActuatorsTelecommandHandlerClass::handleStopActuatorsTelecommand(ESAT_CCSDSPacket telecommand)
{
  (void) telecommand;
  ESAT_ADCS.setRunMode(ESAT_StopActuatorsRunMode);
}

ESAT_StopActuatorsTelecommandHandlerClass ESAT_StopActuatorsTelecommandHandler;
