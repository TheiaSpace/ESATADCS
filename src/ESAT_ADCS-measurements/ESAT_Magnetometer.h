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

#ifndef ESAT_Magnetometer_h
#define ESAT_Magnetometer_h

#include <Arduino.h>

// Magnetometer used for attitude determination.
// Use the global instance ESAT_Magnetometer.
//
// The underlying hardware is the MPU-9250 3-axis gyroscope,
// accelerometer and magnetometer chip from InvenSense mounted on the
// ESAT OBC board.  Communications are done though the OBC I2C bus.
class ESAT_MagnetometerClass
{
  public:
    // True after a read error.  Must be reset manually.
    boolean error;

    // Set up the magnetometer.
    void begin();

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

    // Get the current reading of the magnetic attitude.
    // Set the error flag on error.
    word getReading();

    // Configure the magnetometer in bypass mode.
    // Set the error flag on error.
    void setBypassMode();

    // Start a new reading of the magnetic attitude.
    // Set the error flag on error.
    void startReading();

    // Wait for the reading process to finish.
    // Set the error flag on error.
    void waitForReading();
};

// Global instance of the magnetometer library.
extern ESAT_MagnetometerClass ESAT_Magnetometer;

#endif /* ESAT_Magnetometer_h */