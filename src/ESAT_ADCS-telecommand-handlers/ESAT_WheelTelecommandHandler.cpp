/*
 * Copyright (C) 2017, 2018, 2019 Theia Space, Universidad Politécnica de Madrid
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

#include "ESAT_ADCS-telecommand-handlers/ESAT_WheelTelecommandHandler.h"
#include "ESAT_ADCS.h"
#include "ESAT_ADCS-controllers/ESAT_WheelPIDController.h"
#include "ESAT_ADCS-run-modes/ESAT_WheelResetElectronicSpeedControllerRunMode.h"
#include "ESAT_ADCS-run-modes/ESAT_WheelSetDutyCycleRunMode.h"
#include "ESAT_ADCS-run-modes/ESAT_WheelSetSpeedRunMode.h"

boolean ESAT_WheelTelecommandHandlerClass::handleTelecommand(ESAT_CCSDSPacket telecommand)
{
  telecommand.rewind();
  const ESAT_CCSDSSecondaryHeader secondaryHeader =
    telecommand.readSecondaryHeader();
  switch (secondaryHeader.packetIdentifier)
  {
    case WHEEL_SET_DUTY_CYCLE:
      handleWheelSetDutyCycleTelecommand(telecommand);
      return true;
      break;
    case WHEEL_SET_SPEED:
      handleWheelSetSpeedTelecommand(telecommand);
      return true;
      break;
    case WHEEL_CONTROLLER_SET_PROPORTIONAL_GAIN:
      handleWheelControllerSetProportionalGainTelecommand(telecommand);
      return true;
      break;
    case WHEEL_CONTROLLER_SET_INTEGRAL_GAIN:
      handleWheelControllerSetIntegralGainTelecommand(telecommand);
      return true;
      break;
    case WHEEL_CONTROLLER_SET_DERIVATIVE_GAIN:
      handleWheelControllerSetDerivativeGainTelecommand(telecommand);
      return true;
      break;
    case WHEEL_CONTROLLER_RESET_ERROR_INTEGRAL:
      handleWheelControllerResetErrorIntegralTelecommand(telecommand);
      return true;
      break;
    case WHEEL_RESET_ELECTRONIC_SPEED_CONTROLLER:
      handleWheelResetElectronicSpeedControllerTelecommand(telecommand);
      return true;
      break;
    default:
      return false;
      break;
  }
}

void ESAT_WheelTelecommandHandlerClass::handleWheelSetDutyCycleTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_ADCS.setRunMode(ESAT_WheelSetDutyCycleRunMode);
  ESAT_WheelSetDutyCycleRunMode.dutyCycle = telecommand.readFloat();
}

void ESAT_WheelTelecommandHandlerClass::handleWheelSetSpeedTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_ADCS.setRunMode(ESAT_WheelSetSpeedRunMode);
  ESAT_WheelSetSpeedRunMode.targetSpeed = telecommand.readWord();
}


void ESAT_WheelTelecommandHandlerClass::handleWheelControllerSetProportionalGainTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_WheelPIDController.proportionalGain = telecommand.readFloat();
}

void ESAT_WheelTelecommandHandlerClass::handleWheelControllerSetIntegralGainTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_WheelPIDController.integralGain = telecommand.readFloat();
}

void ESAT_WheelTelecommandHandlerClass::handleWheelControllerSetDerivativeGainTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_WheelPIDController.derivativeGain = telecommand.readFloat();
}

void ESAT_WheelTelecommandHandlerClass::handleWheelControllerResetErrorIntegralTelecommand(ESAT_CCSDSPacket telecommand)
{
  (void) telecommand;
  ESAT_WheelPIDController.resetErrorIntegral();
}

void ESAT_WheelTelecommandHandlerClass::handleWheelResetElectronicSpeedControllerTelecommand(ESAT_CCSDSPacket telecommand)
{
  (void) telecommand;
  ESAT_WheelResetElectronicSpeedControllerRunMode.pendingReset = true;
  ESAT_ADCS.setRunMode(ESAT_WheelResetElectronicSpeedControllerRunMode);
}

ESAT_WheelTelecommandHandlerClass ESAT_WheelTelecommandHandler;
