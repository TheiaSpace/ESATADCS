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

#include "ESAT_ADCS-telecommand-handlers/ESAT_AttitudeTelecommandHandler.h"
#include "ESAT_ADCS.h"
#include "ESAT_ADCS-controllers/ESAT_AttitudePIDController.h"
#include "ESAT_ADCS-run-modes/ESAT_FollowMagneticTargetRunMode.h"
#include "ESAT_ADCS-run-modes/ESAT_FollowSunAngleRunMode.h"

boolean ESAT_AttitudeTelecommandHandlerClass::handleTelecommand(ESAT_CCSDSPacket telecommand)
{
  telecommand.rewind();
  const ESAT_CCSDSSecondaryHeader secondaryHeader =
    telecommand.readSecondaryHeader();
  switch (secondaryHeader.packetIdentifier)
  {
    case FOLLOW_MAGNETIC_TARGET:
      handleFollowMagneticTargetTelecommand(telecommand);
      return true;
      break;
    case FOLLOW_SUN_ANGLE:
      handleFollowSunAngleTelecommand(telecommand);
      return true;
      break;
    case ATTITUDE_CONTROLLER_SET_PROPORTIONAL_GAIN:
      handleAttitudeControllerSetProportionalGainTelecommand(telecommand);
      return true;
      break;
    case ATTITUDE_CONTROLLER_SET_INTEGRAL_GAIN:
      handleAttitudeControllerSetIntegralGainTelecommand(telecommand);
      return true;
      break;
    case ATTITUDE_CONTROLLER_SET_DERIVATIVE_GAIN:
      handleAttitudeControllerSetDerivativeGainTelecommand(telecommand);
      return true;
      break;
    case ATTITUDE_CONTROLLER_RESET_ERROR_INTEGRAL:
      handleAttitudeControllerResetErrorIntegralTelecommand(telecommand);
      return true;
      break;
    case ATTITUDE_CONTROLLER_USE_GYROSCOPE:
      handleAttitudeControllerUseGyroscopeTelecommand(telecommand);
      return true;
      break;
    case ATTITUDE_CONTROLLER_SET_ACTUATOR:
      handleAttitudeControllerSetActuatorTelecommand(telecommand);
      return true;
      break;
    case ATTITUDE_CONTROLLER_SET_DEADBAND:
      handleAttitudeControllerSetDeadbandTelecommand(telecommand);
      return true;
      break;
    case ATTITUDE_CONTROLLER_SET_DETUMBLING_THRESHOLD:
      handleAttitudeControllerSetDetumblingThresholdTelecommand(telecommand);
      return true;
      break;
    default:
      return false;
      break;
  }
}

void ESAT_AttitudeTelecommandHandlerClass::handleFollowMagneticTargetTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_ADCS.setRunMode(ESAT_FollowMagneticTargetRunMode);
  ESAT_AttitudePIDController.targetAngle = (telecommand.readWord() % 360);
}

void ESAT_AttitudeTelecommandHandlerClass::handleFollowSunAngleTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_ADCS.setRunMode(ESAT_FollowSunAngleRunMode);
  ESAT_AttitudePIDController.targetAngle = (telecommand.readWord() % 360);
}


void ESAT_AttitudeTelecommandHandlerClass::handleAttitudeControllerSetProportionalGainTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_AttitudePIDController.proportionalGain = telecommand.readFloat();
}

void ESAT_AttitudeTelecommandHandlerClass::handleAttitudeControllerSetIntegralGainTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_AttitudePIDController.integralGain = telecommand.readFloat();
}

void ESAT_AttitudeTelecommandHandlerClass::handleAttitudeControllerSetDerivativeGainTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_AttitudePIDController.derivativeGain = telecommand.readFloat();
}

void ESAT_AttitudeTelecommandHandlerClass::handleAttitudeControllerResetErrorIntegralTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_AttitudePIDController.resetErrorIntegral();
}

void ESAT_AttitudeTelecommandHandlerClass::handleAttitudeControllerUseGyroscopeTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_AttitudePIDController.useGyroscope = telecommand.readBoolean();
}

void ESAT_AttitudeTelecommandHandlerClass::handleAttitudeControllerSetActuatorTelecommand(ESAT_CCSDSPacket telecommand)
{
  const byte parameter = telecommand.readByte();
  if (parameter > 0)
  {
    ESAT_AttitudePIDController.actuator =
      ESAT_AttitudePIDController.WHEEL;
  }
  else
  {
    ESAT_AttitudePIDController.actuator =
      ESAT_AttitudePIDController.MAGNETORQUER;
  }
}

void ESAT_AttitudeTelecommandHandlerClass::handleAttitudeControllerSetDeadbandTelecommand(ESAT_CCSDSPacket telecommand)
{
  ESAT_AttitudePIDController.errorDeadband = telecommand.readWord();
  ESAT_AttitudePIDController.errorDerivativeDeadband = telecommand.readWord();
}

void ESAT_AttitudeTelecommandHandlerClass::handleAttitudeControllerSetDetumblingThresholdTelecommand(ESAT_CCSDSPacket telecommand)

{
  ESAT_AttitudePIDController.detumblingThreshold = telecommand.readWord();
}

ESAT_AttitudeTelecommandHandlerClass ESAT_AttitudeTelecommandHandler;
