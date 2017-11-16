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

#include "ESAT_ADCS-telecommand-handlers/ESAT_ErrorInjectionTelecommandHandler.h"
#include "ESAT_ADCS.h"
#include "ESAT_ADCS-actuators/ESAT_Magnetorquer.h"
#include "ESAT_ADCS-measurements/ESAT_CoarseSunSensor.h"

boolean ESAT_ErrorInjectionTelecommandHandlerClass::handleTelecommand(ESAT_CCSDSPacket telecommand)
{
  telecommand.rewind();
  const ESAT_CCSDSSecondaryHeader secondaryHeader =
    telecommand.readSecondaryHeader();
  switch (secondaryHeader.packetIdentifier)
  {
    case ERROR_INJECTION_SET_COARSE_SUN_SENSOR_READINGS_SOURCES:
      handleErrorInjectionSetCoarseSunSensorReadingsSources(telecommand);
      return true;
      break;
    case ERROR_INJECTION_CHANGE_MAGNETORQUER_AXES_AND_POLARITIES:
      handleErrorInjectionChangeMagnetorquerAxesAndPolarities(telecommand);
      return true;
      break;
    default:
      return false;
      break;
  }
}

void ESAT_ErrorInjectionTelecommandHandlerClass::handleErrorInjectionSetCoarseSunSensorReadingsSources(ESAT_CCSDSPacket telecommand)
{
  const ESAT_CoarseSunSensorClass::ReadingsSource sourceXPlus =
    ESAT_CoarseSunSensorClass::ReadingsSource(telecommand.readByte());
  const ESAT_CoarseSunSensorClass::ReadingsSource sourceYPlus =
    ESAT_CoarseSunSensorClass::ReadingsSource(telecommand.readByte());
  const ESAT_CoarseSunSensorClass::ReadingsSource sourceXMinus =
    ESAT_CoarseSunSensorClass::ReadingsSource(telecommand.readByte());
  const ESAT_CoarseSunSensorClass::ReadingsSource sourceYMinus =
    ESAT_CoarseSunSensorClass::ReadingsSource(telecommand.readByte());
  ESAT_CoarseSunSensor.setXPlusReadingsSource(sourceXPlus);
  ESAT_CoarseSunSensor.setYPlusReadingsSource(sourceYPlus);
  ESAT_CoarseSunSensor.setXMinusReadingsSource(sourceXMinus);
  ESAT_CoarseSunSensor.setYMinusReadingsSource(sourceYMinus);
}

void ESAT_ErrorInjectionTelecommandHandlerClass::handleErrorInjectionChangeMagnetorquerAxesAndPolarities(ESAT_CCSDSPacket telecommand)
{
  const boolean swapAxes = telecommand.readBoolean();
  const boolean invertXPolarity = telecommand.readBoolean();
  const boolean invertYPolarity = telecommand.readBoolean();
  ESAT_Magnetorquer.swapAxes(swapAxes);
  ESAT_Magnetorquer.invertXPolarity(invertXPolarity);
  ESAT_Magnetorquer.invertYPolarity(invertYPolarity);
}

ESAT_ErrorInjectionTelecommandHandlerClass ESAT_ErrorInjectionTelecommandHandler;
