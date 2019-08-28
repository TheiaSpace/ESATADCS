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

#ifndef ESAT_Gyroscope_h
#define ESAT_Gyroscope_h

#include <Arduino.h>
#include <Wire.h>

// Gyroscope used for measuring the angular speed.
// Use the global instance ESAT_Gyroscope.
//
// The underlying hardware is the MPU-9250 3-axis gyroscope,
// accelerometer and magnetometer chip from InvenSense.
// Communications are done through the I2C bus.
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
    void begin(byte fullScaleConfiguration = FULL_SCALE_2000_DEGREES_PER_SECOND);

    // Configure the sensor bias correction.
    // The gyroscope may have a small bias.
    // Call this function with the satellite perfectly still
    // to estimate the bias for bias correction.
    void configureBiasCorrection();

    // Read the gyroscope.  Return the average of a number of samples.
    // Set the error flag on error.
    int read(unsigned int samples);

  private:
    // I2C address of the gyroscope.
    static const byte ADDRESS = 0x69;

    // Register number of the configuration of the low pass filter.
    static const byte LOW_PASS_FILTER_CONFIGURATION_REGISTER = 26;

    // Register number of the configuration of the gyroscope.
    static const byte CONFIGURATION_REGISTER = 27;

    // Register number of the reading of the gyroscope.
    static const byte GYROSCOPE_READING_REGISTER = 71;

    // Low pass filter configuration.
    static const byte LOW_PASS_FILTER_CONFIGURATION = B00000110;

    // Persistent bias storage.
#ifdef ARDUINO_ESAT_ADCS
    static const int BIAS_EEPROM_ADDRESS = 0;
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
    static const char BIAS_FILENAME[];
#endif /* ARDUINO_ESAT_OBC */

    // Communicate with the magnetometer through this bus.
    // We use a reference: working with bus is the same as
    // working with WireADCS or WireOBC directly.
#ifdef ARDUINO_ESAT_ADCS
    TwoWire& bus = WireADCS;
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
    TwoWire& bus = WireOBC;
#endif /* ARDUINO_ESAT_OBC */

    // Raw reading bias.
    // Used for bias correction.
    int bias;

    // Gain for internal conversions.  Set by setFullScale().
    double gain;

    // Configure the low pass filter of the gyroscope.
    void configureLowPassFilter();

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

    // Read the bias correction from non-volatile memory.
    void readBiasCorrection();

    // Write the bias correction to non-volatile memory.
    void writeBiasCorrection();
};

// Global instance of the gyroscope library.
extern ESAT_GyroscopeClass ESAT_Gyroscope;

#endif /* ESAT_Gyroscope_h */
