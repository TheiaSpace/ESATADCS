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

#ifndef ESAT_DiagnosticsTelecommandHandler_h
#define ESAT_DiagnosticsTelecommandHandler_h

#include <Arduino.h>
#include "ESAT_ADCSTelecommandHandler.h"

// Telecommand handler for diagnostics and troubleshooting.
// Use the global instance ESAT_DiagnosticsTelecommandHandler.
class ESAT_DiagnosticsTelecommandHandlerClass: public ESAT_ADCSTelecommandHandler
{
  public:
    // Either handle the telecommand provided by the packet and return
    // true, or do nothing and return false.
    boolean handleTelecommand(ESAT_CCSDSPacket telecommand);

  private:
    // Available telecommand codes.
    enum TelecommandCodes
    {
      DIAGNOSTICS_SET_COARSE_SUN_SENSOR_READINGS_SOURCES = 0xD0,
      DIAGNOSTICS_CHANGE_MAGNETORQUER_AXES_AND_POLARITIES = 0xD1,
      DIAGNOSTICS_CONFIGURE_GYROSCOPE_BIAS_CORRECTION = 0xD2,
      DIAGNOSTICS_CONFIGURE_MAGNETOMETER_GEOMETRY_CORRECTION = 0xD3,
      DIAGNOSTICS_RECONFIGURE_GYROSCOPE = 0xD4,
      DIAGNOSTICS_CORRECT_GYROSCOPE_BIAS = 0xD5,
      DIAGNOSTICS_CORRECT_MAGNETOMETER_GEOMETRY = 0xD6,
    };

    // Handle the telecommand for setting the coarse sun sensor
    // readings sources.  The telecommand has four 8-bit enumerated
    // parameters:
    // * the source of the +X readings;
    // * the source of the +Y readings;
    // * the source of the -X readings;
    // * the source of the -Y readings.
    // These sources can be:
    // 0: +X sensor.
    // 1: +Y sensor.
    // 2: -X sensor.
    // 3: -Y sensor.
    // 4: 0 % (dark).
    // 5: 100 % (saturated).
    void handleDiagnosticsSetCoarseSunSensorReadingsSources(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for swapping the magnetorquer axes and
    // inverting the polarities.  The telecommand has three 8-bit
    // boolean parameters:
    // * swap (true) or don't swap (false) the magnetorquer axes;
    // * invert (true) or don't invert (false) the X magnetorquer polarity;
    // * invert (true) or don't invert (false) the Y magnetorquer polarity.
    void handleDiagnosticsChangeMagnetorquerAxesAndPolarities(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for configuring the gyroscope bias correction.
    // The telecommand has no parameters.
    void handleDiagnosticsConfigureGyroscopeBiasCorrection(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for configuring the magnetometer geometry correction.
    // The telecommand has eight 16-bit unsigned integer parameters:
    // * the measured magnetic attitude at 0 degrees;
    // * the measured magnetic attitude at 45 degrees;
    // * the measured magnetic attitude at 90 degrees;
    // * the measured magnetic attitude at 135 degrees;
    // * the measured magnetic attitude at 180 degrees;
    // * the measured magnetic attitude at 225 degrees;
    // * the measured magnetic attitude at 270 degrees;
    // * the measured magnetic attitude at 315 degrees.
    void handleDiagnosticsConfigureMagnetometerGeometryCorrection(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for reconfiguring the gyroscope.
    // The telecommand has no parameters.
    void handleDiagnosticsReconfigureGyroscope(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for enabling or disabling the
    // gyroscope bias correction.
    // The telecommand has a boolean (8-bit unsigned integer) parameter:
    // * enable (non-zero) or disable (zero) the correction.
    void handleDiagnosticsCorrectGyroscopeBias(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for enabling or disabling the
    // magnetometer geometry correction.
    // The telecommand has a boolean (8-bit unsigned integer) parameter:
    // * enable (non-zero) or disable (zero) the correction.
    void handleDiagnosticsCorrectMagnetometerGeometry(ESAT_CCSDSPacket telecommand);
};

// Global instance of the diagnostics telecommand handler library.
extern ESAT_DiagnosticsTelecommandHandlerClass ESAT_DiagnosticsTelecommandHandler;

#endif /* ESAT_DiagnosticsTelecommandHandler_h */
