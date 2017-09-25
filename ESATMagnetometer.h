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

#ifndef ESATMagnetometer_h
#define ESATMagnetometer_h

#include <Energia.h>

// Magnetometer used for attitude determination.
// The underlying hardware is the MPU-9250 3-axis
// gyroscope, accelerometer and magnetometer chip
// mounted on the ESATOBC board.
// Communications are done though the OBC I2C bus.
class ESATMagnetometer
{
  public:
    // True if the magnetometer is alive.
    boolean alive;

    // Set up the magnetometer.
    void begin();

    // Read the magnetic attitude (in degrees) relative to North.
    int read();

  private:
    static const byte bypassRegister = 55;
    static const byte chipAddress = 0x69;
    static const byte controlRegister = 0x0A;
    static const byte dataReady = B00000001;
    static const byte dataStatusRegister = 0x02;
    static const byte enableBypass = B00000010;
    static const byte magnetometerAddress = 0x0c;
    static const byte readingRegister = 0x03;
    static const byte singleMeasurementMode = B00000001;

    // Get the current reading of the magnetic attitude.
    int getReading();

    // Configure the magnetometer in bypass mode.
    void setBypassMode();

    // Start a new reading of the magnetic attitude.
    void startReading();

    // Wait for the reading process to finish.
    void waitForReading();
};

extern ESATMagnetometer Magnetometer;

#endif
