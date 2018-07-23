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

#ifndef ESAT_Gyroscope_h
#define ESAT_Gyroscope_h

#include <Arduino.h>
#include <Wire.h>

// Gyroscope used for measuring the angular speed.
// Use the global instance ESAT_Gyroscope.
//
// The underlying hardware is the MPU-9250 3-axis gyroscope,
// accelerometer and magnetometer chip from InvenSense mounted on the
// ESAT OBC board.  Communications are done though the OBC I2C bus.
class ESAT_GyroscopeClass
{
  public:
    // Full scale configurations.
    static const byte FULL_SCALE_250_DEGREES_PER_SECOND = B00;
    static const byte FULL_SCALE_500_DEGREES_PER_SECOND = B01;
    static const byte FULL_SCALE_1000_DEGREES_PER_SECOND = B10;
    static const byte FULL_SCALE_2000_DEGREES_PER_SECOND = B11;

    // True after a read error.  Must be reset manually.
    boolean error;

    // Set up the gyroscope with a full scale configuration given by
    // one of the FULL_SCALE_X_DEGREES_PER_SECOND constants.
    // Set the error flag on error.
    void begin(byte fullScaleConfiguration);

    // Read the gyroscope.  Return the average of a number of samples.
    // Set the error flag on error.
    int read(unsigned int samples);

  private:
    // I2C address of the gyroscope.
    static const byte ADDRESS = 0x69;

    // Register number of the configuration of the gyroscope.
    static const byte CONFIGURATION_REGISTER = 27;

    // Register number of the reading of the gyroscope.
    static const byte GYROSCOPE_READING_REGISTER = 71;

    // Communicate with the magnetometer through this bus.
    TwoWire* bus;

    // Gain for internal conversions.  Set by setFullScale().
    double gain;

    // Configure the range of measurement of the gyroscope according
    // to the full scale configuration.
    // Set the error flag on error.
    void configureRange(byte fullScaleConfiguration);

    // Read a raw sample.
    // Set the error flag on error.
    int readRawSample();

    // Set the gain for internal conversions according to the full
    // scale configuration.
    void setGain(byte fullScaleConfiguration);
};

// Global instance of the gyroscope library.
extern ESAT_GyroscopeClass ESAT_Gyroscope;

#endif /* ESAT_Gyroscope_h */
