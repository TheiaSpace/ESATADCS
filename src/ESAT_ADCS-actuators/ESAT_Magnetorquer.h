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

#ifndef ESAT_Magnetorquer_h
#define ESAT_Magnetorquer_h

#include <Arduino.h>

// Magnetorquers used for attitude control.
// Use the global instance ESAT_Magnetorquer.
//
// There are two magnetorquers: one aligned with the X-axis and one
// aligned with the Y-axis.  These magnetorquers are electromagnets
// whose power (on/off) and polarity (positive/negative) can be 
// switched thanks to a H-bridge.  Together they form
// a 2D magnetorquer that works like a compass needle that can be 
// magnetized in 1 of 4 perpendicular directions at will by changing
// the polarities of the X-axis and Y-axis 1D magnetorquers.  Like a
// compass needle tends to point to the North, so will do the
// satellite (one corner of the satellite that depends on the X-axis
// and Y-axis 1D magnetorquer polarities) when the 2D magnetorquer is
// switched on.
class ESAT_MagnetorquerClass
{
  public:
    // Magnetorquer polarity: either negative or positive.
    enum Polarity
    {
      NEGATIVE = 0,
      POSITIVE = 1,
    };

    // Set up the magnetorquers.
    void begin();

    // Return true if the magnetorquer driver is powered up; otherwise
    // return false.
    boolean readEnable() const;

    // Return the polarity of the X-axis magnetorquer.
    Polarity readX() const;

    // Return the polarity of the Y-axis magnetorquer.
    Polarity readY() const;

    // Power up or down the magnetorquer driver.
    void writeEnable(boolean enable);

    // Set the polarity of the X-axis magnetorquer.
    void writeX(Polarity polarity);

    // Set the polarity of the Y-axis magnetorquer.
    void writeY(Polarity polarity);

  private:
    // This pin enables the X magnetorquer power.
    static const int PIN_ENABLE_X = ENMTQX;

    // This pin enables the Y magnetorquer power.
    static const int PIN_ENABLE_Y = ENMTQY;

    // This pin selects the X magnetorquer polarity.
    static const int PIN_X_POLARITY = MTQX;

    // This pin selects the Y magnetorquer polarity.
    static const int PIN_Y_POLARITY = MTQY;

    // True if the magnetorquer driver is powered up; false otherwise.
    boolean enable;

    // Polarity of the X-axis magnetorquer.
    Polarity xPolarity;

    // Polarity of the Y-axis magnetorquer.
    Polarity yPolarity;

    // Configure the control pins.
    void configurePins();
};

// Global instance of the magnetorquer library.
extern ESAT_MagnetorquerClass ESAT_Magnetorquer;

#endif /* ESAT_Magnetorquer_h */
