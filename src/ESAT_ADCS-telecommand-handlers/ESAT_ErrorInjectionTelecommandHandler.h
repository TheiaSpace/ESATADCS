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

#ifndef ESAT_ErrorInjectionTelecommandHandler_h
#define ESAT_ErrorInjectionTelecommandHandler_h

#include <Arduino.h>
#include "ESAT_ADCSTelecommandHandler.h"

// Telecommand handler for simulated error injection.
// Use the global instance ESAT_ErrorInjectionTelecommandHandler.
class ESAT_ErrorInjectionTelecommandHandlerClass: public ESAT_ADCSTelecommandHandler
{
  public:
    // Either handle the telecommand provided by the packet and return
    // true, or do nothing and return false.
    boolean handleTelecommand(ESAT_CCSDSPacket telecommand);

  private:
    // Available telecommand codes.
    enum TelecommandCodes
    {
      ERROR_INJECTION_SET_COARSE_SUN_SENSOR_SOURCES = 0xE0,
      ERROR_INJECTION_SWAP_MAGNETORQUER_AXES_AND_POLARITIES = 0xE1,
    };

    // Handle the telecommand for setting the coarse sun sensor
    // reading sources.  The telecommand has four 8-bit enumerated
    // parameters:
    // * the source of the +X reading;
    // * the source of the +Y reading;
    // * the source of the -X reading;
    // * the source of the -Y reading.
    // These sources can be:
    // 0: +X sensor.
    // 1: +Y sensor.
    // 2: -X sensor.
    // 3: -Y sensor.
    // 4: 0 % (dark).
    // 5: 100 % (saturated).
    void handleErrorInjectionSetCoarseSunSensorSources(ESAT_CCSDSPacket telecommand);

    // Handle the telecommand for swapping the magnetorquer axes and
    // polarities.  The telecommand has three 8-bit boolean parameters:
    // * swap (true) or don't swap (false) the magnetorquer axes;
    // * invert (true) or don't invert (false) the X magnetorquer polarity;
    // * invert (true) or don't invert (false) the Y magnetorquer polarity.
    void handleErrorInjectionSwapMagnetorquerAxesAndPolarities(ESAT_CCSDSPacket telecommand);
};

// Global instance of the wheel telecommand handler library.
extern ESAT_ErrorInjectionTelecommandHandlerClass ESAT_ErrorInjectionTelecommandHandler;

#endif /* ESAT_ErrorInjectionTelecommandHandler_h */
