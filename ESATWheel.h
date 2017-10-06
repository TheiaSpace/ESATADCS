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
    // Set up the wheel controller.
    void begin();

    // Set the speed of the wheel.
    void write(word rpm);

    // Set the duty cycle of the electronic speed controller.
    void writeDutyCycle(byte dutyCycle);

  private:
    // Maximum and minimum pulse widths
    // for the electronic speed controller.
    static const unsigned int MAXIMUM = 1860;
    static const unsigned int MINIMUM = 1060;

    // The electronic speed controller is attached to this pin.
    static const int PIN = PWM;

    // Power line switch on message.
    static const byte POWER_LINE_ADDRESS = 2;
    static const byte POWER_LINE_TELECOMMAND_PRIMARY_HEADER_REGISTER = 0;
    static const byte POWER_LINE_TELECOMMAND_PACKET_DATA_REGISTER = 1;
    static const byte POWER_LINE_IDENTIFIER = 2;
    static const byte POWER_LINE_MAJOR_VERSION_NUMBER = 2;
    static const byte POWER_LINE_MINOR_VERSION_NUMBER = 0;
    static const byte POWER_LINE_PATCH_VERSION_NUMBER = 0;
    static const byte POWER_LINE_COMMAND_CODE = 1;
    static const byte POWER_LINE_COMMAND_BUFFER_LENGTH = 11;

    // RPM to duty cycle calibration coefficients.
    float calibration[2];

    // Servo object for commanding the electronic speed controller.
    Servo electronicSpeedController;

    // Run the programming sequence of the electronic speed controller.
    void programElectronicSpeedController();

    // Switch on or off the electronic speed controller.
    void switchElectronicSpeedController(boolean on);
};

extern ESATWheel Wheel;

#endif
