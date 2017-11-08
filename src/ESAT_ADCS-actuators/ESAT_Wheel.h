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

#ifndef ESAT_Wheel_h
#define ESAT_Wheel_h

#include <Arduino.h>
#include <ESAT_CCSDSSecondaryHeader.h>
#include <Servo.h>

// Reaction wheel.
// Use the global instance ESAT_Wheel.
//
// The ESAT has a heavy flywheel attached to a motor.  Changing the
// rotational speed of the wheel changes the rotational speed of the
// satellite in the opposite direction.
class ESAT_WheelClass
{
  public:
    // Set up the wheel controller.
    void begin();

    // Return the duty cycle of the electronic speed controller.  The
    // duty cycle is a signed percentage: it should go from -100 % to
    // 0 % for negative (clockwise) speed and from 0 % to 100 % for
    // positive (counterclockwise) speed.
    float readDutyCycle();

    // Set the speed of the wheel in revolutions per minute: negative
    // for clockwise rotation and positive for counterclockwise
    // rotation.
    void writeSpeed(int rpm);

    // Set the duty cycle of the electronic speed controller.  The
    // duty cycle is a signed percentage: it should go from -100 % to
    // 0 % for negative (clockwise) speed and from 0 % to 100 % for
    // positive (counterclockwise) speed.
    void writeDutyCycle(float dutyCycle);

  private:
    // Duty cycle percentage point per wheel speed RPM.
    static constexpr float DUTY_CYCLE_PER_RPM = 0.015;

    // Maximum and minimum pulse widths
    // for the electronic speed controller.
    static const word MAXIMUM_PULSE_WIDTH = 1860;
    static const word MINIMUM_PULSE_WIDTH = 1060;

    // Maximum allowed wheel speed in rpm.
    static const int MAXIMUM_WHEEL_SPEED = 8000;

    // The electronic speed controller is attached to this pin.
    static const int PIN = PWM;

    // Power line switch on message.
    static const byte POWER_LINE_ADDRESS = 1;
    static const byte POWER_LINE_TELECOMMAND_PRIMARY_HEADER_REGISTER = 0;
    static const byte POWER_LINE_TELECOMMAND_PACKET_DATA_REGISTER = 1;
    static const byte POWER_LINE_IDENTIFIER = 1;
    static const byte POWER_LINE_MAJOR_VERSION_NUMBER = 2;
    static const byte POWER_LINE_MINOR_VERSION_NUMBER = 0;
    static const byte POWER_LINE_PATCH_VERSION_NUMBER = 0;
    static const byte POWER_LINE_COMMAND_CODE = 0x11;
    static const byte POWER_LINE_MILLISECONDS_AFTER_WRITES = 1;
    static const byte POWER_LINE_ATTEMPTS = 10;
    static const byte POWER_LINE_MILLISECONDS_BETWEEN_ATTEMPTS = 10;

    // Duty cycle of the electronic speed controller.
    // The duty cycle is a signed percentage: it should go from -100 %
    // to 0 % for negative speed and from 0 % to 100 % for positive
    // speed.
    float dutyCycle;

    // Servo object for commanding the electronic speed controller.
    Servo electronicSpeedController;

    // Run the calibration sequence of the electronic speed controller.
    void calibrateElectronicSpeedController();

    // Constrain the duty cycle constrained to lie
    // between -100 % and +100 %.
    float constrainDutyCycle(float dutyCycle);

    // Constrain the wheel speed to lie between -MAXIMUM_WHEEL_SPEED
    // and MAXIMUM_WHEEL_SPEED.
    word constrainSpeed(int rpm);

    // Switch on or off the electronic speed controller.
    void switchElectronicSpeedController(boolean on);
};

extern ESAT_WheelClass ESAT_Wheel;

#endif /* ESAT_Wheel_h */
