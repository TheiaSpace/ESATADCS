/*
 * Copyright (C) 2017, 2019 Theia Space, Universidad Politécnica de Madrid
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

#include "ESAT_ADCS-telecommand-handlers/ESAT_DiagnosticsTelecommandHandler.h"
#include "ESAT_ADCS.h"
#include "ESAT_ADCS-actuators/ESAT_Magnetorquer.h"
#include "ESAT_ADCS-measurements/ESAT_CoarseSunSensor.h"
#include "ESAT_ADCS-measurements/ESAT_Gyroscope.h"
#include "ESAT_ADCS-measurements/ESAT_Magnetometer.h"

boolean ESAT_DiagnosticsTelecommandHandlerClass::handleTelecommand(ESAT_CCSDSPacket telecommand)
{
  telecommand.rewind();
  const ESAT_CCSDSSecondaryHeader secondaryHeader =
    telecommand.readSecondaryHeader();
  switch (secondaryHeader.packetIdentifier)
  {
    case DIAGNOSTICS_SET_COARSE_SUN_SENSOR_READINGS_SOURCES:
      handleDiagnosticsSetCoarseSunSensorReadingsSources(telecommand);
      return true;
      break;
    case DIAGNOSTICS_CHANGE_MAGNETORQUER_AXES_AND_POLARITIES:
      handleDiagnosticsChangeMagnetorquerAxesAndPolarities(telecommand);
      return true;
      break;
    case DIAGNOSTICS_CONFIGURE_GYROSCOPE_BIAS_CORRECTION:
      handleDiagnosticsConfigureGyroscopeBiasCorrection(telecommand);
      return true;
      break;
    case DIAGNOSTICS_CONFIGURE_MAGNETOMETER_GEOMETRY_CORRECTION:
      handleDiagnosticsConfigureMagnetometerGeometryCorrection(telecommand);
      return true;
      break;
    case DIAGNOSTICS_RECONFIGURE_GYROSCOPE:
      handleDiagnosticsReconfigureGyroscope(telecommand);
      return true;
      break;
    default:
      return false;
      break;
  }
}

void ESAT_DiagnosticsTelecommandHandlerClass::handleDiagnosticsSetCoarseSunSensorReadingsSources(ESAT_CCSDSPacket telecommand)
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

void ESAT_DiagnosticsTelecommandHandlerClass::handleDiagnosticsChangeMagnetorquerAxesAndPolarities(ESAT_CCSDSPacket telecommand)
{
  const boolean swapAxes = telecommand.readBoolean();
  const boolean invertXPolarity = telecommand.readBoolean();
  const boolean invertYPolarity = telecommand.readBoolean();
  ESAT_Magnetorquer.swapAxes(swapAxes);
  ESAT_Magnetorquer.invertXPolarity(invertXPolarity);
  ESAT_Magnetorquer.invertYPolarity(invertYPolarity);
}

void ESAT_DiagnosticsTelecommandHandlerClass::handleDiagnosticsConfigureGyroscopeBiasCorrection(ESAT_CCSDSPacket telecommand)
{
  (void) telecommand; // Unused.
  ESAT_Gyroscope.configureBiasCorrection();
}

void ESAT_DiagnosticsTelecommandHandlerClass::handleDiagnosticsConfigureMagnetometerGeometryCorrection(ESAT_CCSDSPacket telecommand)
{
  const word measurement0 = telecommand.readWord();
  const word measurement45 = telecommand.readWord();
  const word measurement90 = telecommand.readWord();
  const word measurement135 = telecommand.readWord();
  const word measurement180 = telecommand.readWord();
  const word measurement225 = telecommand.readWord();
  const word measurement270 = telecommand.readWord();
  const word measurement315 = telecommand.readWord();
  ESAT_Magnetometer.configureGeometryCorrection(measurement0,
                                                measurement45,
                                                measurement90,
                                                measurement135,
                                                measurement180,
                                                measurement225,
                                                measurement270,
                                                measurement315);
}

void ESAT_DiagnosticsTelecommandHandlerClass::handleDiagnosticsReconfigureGyroscope(ESAT_CCSDSPacket telecommand)
{
  (void) telecommand; // Unused.
  ESAT_Gyroscope.begin();
}

ESAT_DiagnosticsTelecommandHandlerClass ESAT_DiagnosticsTelecommandHandler;
