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

#include "ESAT_MagnetorquerTelecommandHandler.h"
#include "ESAT_ADCS.h"
#include "ESAT_MagnetorquerApplyMaximumTorqueRunMode.h"
#include "ESAT_MagnetorquerDemagnetizeRunMode.h"
#include "ESAT_MagnetorquerEnableRunMode.h"
#include "ESAT_MagnetorquerSetXPolarityRunMode.h"
#include "ESAT_MagnetorquerSetYPolarityRunMode.h"
#include "ESAT_Magnetorquer.h"

boolean ESAT_MagnetorquerTelecommandHandlerClass::handleTelecommand(ESAT_CCSDSPacket telecommand)
{
  telecommand.rewind();
  const ESAT_CCSDSSecondaryHeader secondaryHeader =
    telecommand.readSecondaryHeader();
  switch (secondaryHeader.packetIdentifier)
  {
    case MAGNETORQUER_ENABLE:
      handleMagnetorquerEnableTelecommand(telecommand);
      return true;
      break;
    case MAGNETORQUER_SET_X_POLARITY:
      handleMagnetorquerSetXPolarityTelecommand(telecommand);
      return true;
      break;
    case MAGNETORQUER_SET_Y_POLARITY:
      handleMagnetorquerSetYPolarityTelecommand(telecommand);
      return true;
      break;
    case MAGNETORQUER_APPLY_MAXIMUM_TORQUE:
      handleMagnetorquerApplyMaximumTorqueTelecommand(telecommand);
      return true;
      break;
    case MAGNETORQUER_DEMAGNETIZE:
      handleMagnetorquerDemagnetizeTelecommand(telecommand);
      return true;
      break;
    default:
      return false;
      break;
  }
}

void ESAT_MagnetorquerTelecommandHandlerClass::handleMagnetorquerEnableTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_ADCS.setRunMode(ESAT_MagnetorquerEnableRunMode);
  ESAT_MagnetorquerEnableRunMode.enable = telecommand.readBoolean();
}

void ESAT_MagnetorquerTelecommandHandlerClass::handleMagnetorquerSetXPolarityTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_ADCS.setRunMode(ESAT_MagnetorquerSetXPolarityRunMode);
  if (telecommand.readByte() == 0)
  {
    ESAT_MagnetorquerSetXPolarityRunMode.polarity =
      ESAT_Magnetorquer.NEGATIVE;
  }
  else
  {
    ESAT_MagnetorquerSetXPolarityRunMode.polarity =
      ESAT_Magnetorquer.NEGATIVE;
  }
}

void ESAT_MagnetorquerTelecommandHandlerClass::handleMagnetorquerSetYPolarityTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_ADCS.setRunMode(ESAT_MagnetorquerSetYPolarityRunMode);
  if (telecommand.readByte() == 0)
  {
    ESAT_MagnetorquerSetYPolarityRunMode.polarity =
      ESAT_Magnetorquer.NEGATIVE;
  }
  else
  {
    ESAT_MagnetorquerSetYPolarityRunMode.polarity =
      ESAT_Magnetorquer.NEGATIVE;
  }
}

void ESAT_MagnetorquerTelecommandHandlerClass::handleMagnetorquerApplyMaximumTorqueTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_ADCS.setRunMode(ESAT_MagnetorquerApplyMaximumTorqueRunMode);
  if (telecommand.readByte() == 0)
  {
    ESAT_MagnetorquerApplyMaximumTorqueRunMode.mode =
      ESAT_MagnetorquerApplyMaximumTorqueRunMode.ROTATE_CLOCKWISE;
  }
  else
  {
    ESAT_MagnetorquerApplyMaximumTorqueRunMode.mode =
      ESAT_MagnetorquerApplyMaximumTorqueRunMode.ROTATE_COUNTERCLOCKWISE;
  }
}

void ESAT_MagnetorquerTelecommandHandlerClass::handleMagnetorquerDemagnetizeTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_ADCS.setRunMode(ESAT_MagnetorquerDemagnetizeRunMode);
  ESAT_MagnetorquerDemagnetizeRunMode.cycles = telecommand.readByte();
}

ESAT_MagnetorquerTelecommandHandlerClass ESAT_MagnetorquerTelecommandHandler;
