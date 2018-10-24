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

    // Invert (if the argument is true) or don't invert (if the
    // argument is false; default value) the polarity of the X
    // magnetorquer.  This is useful for diagnostics and
    // troubleshooting.
    void invertXPolarity(boolean invert);

    // Invert (if the argument is true) or don't invert (if the
    // argument is false; default value) the polarity of the Y
    // magnetorquer.  This is useful for diagnostics and
    // troubleshooting.
    void invertYPolarity(boolean invert);

    // Return true if the magnetorquer driver is powered up; otherwise
    // return false.
    boolean readEnable() const;

    // Return the polarity of the X-axis magnetorquer.
    Polarity readX() const;

    // Return the polarity of the Y-axis magnetorquer.
    Polarity readY() const;

    // Swap (if the argument is true) or don't swap (if the argument
    // is false; default value) the axes of the magnetorquer.  This is
    // useful for diagnostics and troubleshooting.
    void swapAxes(boolean swap);

    // Power up or down the magnetorquer driver.
    void writeEnable(boolean enable);

    // Set the polarity of the X-axis magnetorquer.
    void writeX(Polarity polarity);

    // Set the polarity of the Y-axis magnetorquer.
    void writeY(Polarity polarity);

  private:
    // True if the magnetorquer driver is powered up; false otherwise.
    boolean enable;

    // True if the the X magnetorquer polarity is inverted; false
    // otherwise.  This is useful for diagnostics and troubleshooting.
    boolean invertXPolarityLevel;

    // True if the the Y magnetorquer polarity is inverted; false
    // otherwise.  This is useful for diagnostics and troubleshooting.
    boolean invertYPolarityLevel;

    // True if the axes of the magnetorquer are swapped; false
    // otherwise.  This is useful for diagnostics and troubleshooting.
    boolean swapPolarityPins;

    // Polarity of the X-axis magnetorquer.
    Polarity xPolarity;

    // Polarity of the Y-axis magnetorquer.
    Polarity yPolarity;

    // Configure the control pins.
    void configurePins();

    // Invert a digital line level: from HIGH to LOW and from LOW to
    // HIGH.
    int invertLevel(int level) const;

#ifdef ARDUINO_ESAT_OBC
    // Return the pin used for selecting the X magnetorquer polarity.
    // It can be the wrong one if the polarity pins are swapped; this
    // is useful for diagnostics and troubleshooting.
    int pinXPolarity() const;
#endif /* ARDUINO_ESAT_OBC */

#ifdef ARDUINO_ESAT_ADCS
    // Return the pin used for selecting the X magnetorquer polarity
    // (negative).
    // It can be the wrong one if the polarity pins are swapped; this
    // is useful for diagnostics and troubleshooting.
    int pinXPolarityMinus() const;
#endif /* ARDUINO_ESAT_ADCS */

#ifdef ARDUINO_ESAT_ADCS
    // Return the pin used for selecting the X magnetorquer polarity
    // (positive).
    // It can be the wrong one if the polarity pins are swapped; this
    // is useful for diagnostics and troubleshooting.
    int pinXPolarityPlus() const;
#endif /* ARDUINO_ESAT_ADCS */

#ifdef ARDUINO_ESAT_OBC
    // Return the pin used for selecting the Y magnetorquer polarity.
    // It can be the wrong one if the polarity pins are swapped; this
    // is useful for diagnostics and troubleshooting.
    int pinYPolarity() const;
#endif /* ARDUINO_ESAT_OBC */

#ifdef ARDUINO_ESAT_ADCS
    // Return the pin used for selecting the Y magnetorquer polarity
    // (negative).
    // It can be the wrong one if the polarity pins are swapped; this
    // is useful for diagnostics and troubleshooting.
    int pinYPolarityMinus() const;
#endif /* ARDUINO_ESAT_ADCS */

#ifdef ARDUINO_ESAT_ADCS
    // Return the pin used for selecting the Y magnetorquer polarity
    // (positive).
    // It can be the wrong one if the polarity pins are swapped; this
    // is useful for diagnostics and troubleshooting.
    int pinYPolarityPlus() const;
#endif /* ARDUINO_ESAT_ADCS */

    // Return the line level corresponding to a magnetorquer polarity.
    int polarityLevel(Polarity polarity) const;

    // Return the line level for selecting the X magnetorquer
    // polarity.  It can be the wrong one if the X polarity is
    // swapped; this is useful for diagnostics and troubleshooting.
    int polarityXLevel() const;

    // Return the line level for selecting the X magnetorquer
    // polarity.  It can be the wrong one if the Y polarity is
    // swapped; this is useful for diagnostics and troubleshooting.
    int polarityYLevel() const;
};

// Global instance of the magnetorquer library.
extern ESAT_MagnetorquerClass ESAT_Magnetorquer;

#endif /* ESAT_Magnetorquer_h */
