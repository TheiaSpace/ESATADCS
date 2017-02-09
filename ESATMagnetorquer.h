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

#ifndef ESATMagnetorquer_h
#define ESATMagnetorquer_h

#include <Energia.h>

// Magnetorquers used for attitude control.
class ESATMagnetorquer
{
  public:
    enum
    {
      negative = LOW,
      positive = HIGH,
    } Polarity;

    // Set up the magnetorquers.
    void begin();

    // Power up or down the magnetorquer driver.
    void writeEnable(boolean enable);

    // Set the polarity of the X-axis magnetorquer.
    void writeX(int polarity);

    // Set the polarity of the Y-axis magnetorquer.
    void writeY(int polarity);

  private:
    // This pin enables the X magnetorquer.
    static const int pinEnableX = 79;

    // This pin enables the X magnetorquer.
    static const int pinEnableY = 78;

    // This pin activates the X magnetorquer.
    static const int pinX = 80;

    // This pin activates the Y magnetorquer.
    static const int pinY = 1;

    // Configure the control pins.
    void configurePins();
};

extern ESATMagnetorquer Magnetorquer;

#endif