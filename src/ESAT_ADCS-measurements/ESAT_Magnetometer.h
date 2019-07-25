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

#ifndef ESAT_Magnetometer_h
#define ESAT_Magnetometer_h

#include <Arduino.h>
#include <Wire.h>

// Magnetometer used for attitude determination.
// Use the global instance ESAT_Magnetometer.
//
// The underlying hardware is the MPU-9250 3-axis gyroscope,
// accelerometer and magnetometer chip from InvenSense.
// Communications are done through the I2C bus.
class ESAT_MagnetometerClass
{
  public:
    // True after a read error.  Must be reset manually.
    boolean error;

    // Set up the magnetometer.
    void begin();

    // Configure the geometry correction based on measured attitudes
    // at several actual attitudes.
    // The measured angle of the magnetic field can differ from the
    // actual magnetic attitude due to several factors (the
    // magnetometer isn't in the exact axis of rotation, the Earth
    // magnetic field simulator magnets may be slightly misaligned…),
    // so it's necessary to correct for these geometrical factors
    // to achieve good attitude measurements.  The geometry
    // correction takes into account the magnetic attitude measured
    // at 0 degrees, 45 degrees, 90 degrees, 135 degrees, 180 degrees,
    // 225 degrees, 270 degrees and 315 degrees.
    void configureGeometryCorrection(word measurement0,
                                     word measurement45,
                                     word measurement90,
                                     word measurement135,
                                     word measurement180,
                                     word measurement225,
                                     word measurement270,
                                     word measurement315);

    // Read the magnetic attitude (in degrees) relative to North:
    // the counterclockwise angle from the +X axis of the satellite
    // to the North direction or the clockwise angle from the North
    // direction to the +X axis of the satellite.
    // The attitude goes from 0 degrees to 359 degrees.
    word read();

  private:
    // I2C addresses, registers and flags used when communicating with
    // the magnetometer chip.
    static const byte BYPASS_REGISTER = 55;
    static const byte CHIP_ADDRESS = 0x69;
    static const byte CONTROL_REGISTER = 0x0A;
    static const byte DATA_READY = B00000001;
    static const byte DATA_STATUS_REGISTER = 0x02;
    static const byte ENABLE_BYPASS = B00000010;
    static const byte MAGNETOMETER_ADDRESS = 0x0C;
    static const byte READING_REGISTER = 0x03;
    static const byte SINGLE_MEASUREMENT_MODE = B00000001;

    // Number of positions of the geometry correction process.
    static const byte GEOMETRY_CORRECTION_POSITIONS = 8;

    // Persistent geometry correction storage.
#ifdef ARDUINO_ESAT_ADCS
    static const int GEOMETRY_EEPROM_ADDRESS = 2;
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
    static const char GEOMETRY_FILENAME[];
#endif /* ARDUINO_ESAT_ADCS */

    // Communicate with the magnetometer through this bus.
    // We use a reference: working with bus is the same as
    // working with WireADCS or WireOBC directly.
#ifdef ARDUINO_ESAT_ADCS
    TwoWire& bus = WireADCS;
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
    TwoWire& bus = WireOBC;
#endif /* ARDUINO_ESAT_OBC */

    // Geometry correction coefficients.
    word fieldAngles[GEOMETRY_CORRECTION_POSITIONS + 2];

    // Return the difference between two angles, normalised between
    // -180 degrees and 180 degrees.
    float angleDifference(float minuend, float subtrahend) const;

    // Return the magnetic cattitude angle in degres sas deduced from
    // the magnetic field components.  The attitude angle ranges from
    // 0 degrees to 359 degrees.
    word computeAttitude(float xField,
                         float yField) const;

    // Get the current reading of the magnetic attitude.
    // Set the error flag on error.
    word getReading();

    // Return the attitude angle normalised from 0 degrees to 359
    // degrees.
    word normaliseAttitude(int attitude) const;

    // Read the geometry correction from persistent storage.
    void readGeometryCorrection();

    // Configure the magnetometer in bypass mode.
    // Set the error flag on error.
    void setBypassMode();

    // Start a new reading of the magnetic attitude.
    // Set the error flag on error.
    void startReading();

    // Wait for the reading process to finish.
    // Set the error flag on error.
    void waitForReading();

    // Write the geometry correction to persistent storage.
    void writeGeometryCorrection();
};

// Global instance of the magnetometer library.
extern ESAT_MagnetometerClass ESAT_Magnetometer;

#endif /* ESAT_Magnetometer_h */
