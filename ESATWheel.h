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

#ifndef ESATWheel_h
#define ESATWheel_h

#include <Energia.h>
#include <Servo.h>

// Rotating wheel.
class ESATWheel
{
  public:
    // Coefficients of the calibration curve.
    float calibration[3];

    // Set up the wheel controller.
    void begin();

    // Set the calibration curve to its default value.
    void defaultCalibration();

    // Load the calibration curve from flash.
    void loadCalibration();

    // Save the calibration curve to flash.
    void saveCalibration();

    // Set the speed of the wheel.
    void write(int rpm);

    // Set the duty cycle of the electronic speed controller.
    void writeDutyCycle(byte dutyCycle);

  private:
    // Maximum and minimum pulse widths
    // for the electronic speed controller.
    static const unsigned int maximum = 1860;
    static const unsigned int minimum = 1060;

    // The electronic speed controller is attached to this pin.
    static const int pin = 23;

    static const byte programmingAddress = 2;
    static const byte programmingRegister = 1;
    static const byte programmingMessage = 0;

    // Servo object for commanding the electronic speed controller.
    Servo electronicSpeedController;

    // Run the programming sequence of the electronic speed controller.
    void programElectronicSpeedController();
};

extern ESATWheel Wheel;

#endif
