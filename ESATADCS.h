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

#ifndef ESATADCS_h
#define ESATADCS_h
#include <Energia.h>

class ESATADCS
{
  public:
    // True if the inertial measurement unit is alive.
    boolean inertialMeasurementUnitAlive;

    // Get all ADCS subsystems ready.
    void begin();

    // Handle a telecommand.
    void handleCommand(int commandCode, String parameters);

    // Return the telemetry of the ADCS.
    String readTelemetry();

    void update();

  private:
    // Command codes.
    enum CommandCode
    {
      MOTOR_DUTY_COMMAND = 0x1,
      PID_CONFIGURATION_COMMAND = 0x2,
      ENABLE_MAGNETORQUER_X_AND_MAGNETORQUER_Y_COMMAND = 0x3,
      MAGNETORQUER_X_POLARITY_COMMAND = 0x4,
      MAGNETORQUER_Y_POLARITY_COMMAND = 0x5,
      FOLLOW_SUN_COMMAND = 0x6,
      FOLLOW_MAGNETOMETER_COMMAND = 0x7,
      SET_WHEEL_SPEED_COMMAND = 0x8,
      MAXIMUM_MAGNETIC_TORQUE_COMMAND = 0x9,
      WHEEL_PID_CONFIGURATION_COMMAND = 0x10,
      DEMAGNETIZE_COMMAND = 0x11,
      WHEEL_OR_MAGNETORQUER_COMMAND = 0x13,
      WHEEL_CALIBRATION1_COMMAND = 0x21,
      WHEEL_CALIBRATION2_COMMAND = 0x22,
      WHEEL_CALIBRATION3_COMMAND = 0x23,
      FLASH_WRITE_COMMAND = 0x24,
      FLASH_WRITE_DEFAULTS_COMMAND = 0x25
    };

    // Run codes.
    enum RunCode
    {
      REST = 0,
      SET_MOTOR_DUTY = 1,
      ENABLE_MAGNETORQUER_X_AND_MAGNETORQUER_Y = 3,
      FOLLOW_SUN = 6,
      FOLLOW_MAGNETOMETER = 7,
      SET_WHEEL_SPEED = 8,
      MAXIMUM_MAGNETIC_TORQUE = 9,
      DEMAGNETIZE = 11,
    };

    float attitudeDerivativeGain;
    float attitudeErrorIntegral;
    float attitudeIntegralGain;
    float attitudeProportionalGain;
    int demagnetizationIterations;
    boolean enableMagnetorquerDriver;
    int magneticAngle;
    int magnetorquerXPolarity;
    int magnetorquerYPolarity;
    int oldWheelSpeedError;
    int rotationalSpeed;
    enum RunCode runCode;
    int sunAngle;
    int targetAttitude;
    boolean targetMagnetorquerDirection;
    int targetWheelSpeed;
    boolean useWheel;
    float wheelDerivativeGain;
    float wheelIntegralGain;
    float wheelProportionalGain;
    unsigned int wheelSpeed;
    float wheelSpeedErrorIntegral;

    // Light blink sequence performed at startup.
    void blinkSequence();

    // Commands.
    void handleMotorDutyCommand(String parameters);
    void handlePIDConfigurationCommand(String parameters);
    void handleEnableMagnetorquerXAndMagnetorquerYCommand(String parameters);
    void handleMagnetorquerXPolarityCommand(String parameters);
    void handleMagnetorquerYPolarityCommand(String parameters);
    void handleFollowSunCommand(String parameters);
    void handleFollowMagnetometerCommand(String parameters);
    void handleSetWheelSpeedCommand(String parameters);
    void handleMaximumMagneticTorqueCommand(String parameters);
    void handleWheelPIDConfigurationCommand(String parameters);
    void handleDemagnetizeCommand(String parameters);
    void handleWheelOrMagnetorquerCommand(String parameters);
    void handleWheelCalibration1Command(String parameters);
    void handleWheelCalibration2Command(String parameters);
    void handleWheelCalibration3Command(String parameters);
    void handleFlashWriteCommand(String parameters);
    void handleFlashWriteDefaultsCommand(String parameters);

    // Read the sensors needed for attitude determination and control.
    void readSensors();

    // Actuate according to the current run code.
    void run();

    // Attitude control loop.
    void runAttitudeControlLoop(int currentAttitude);

    // Attitude control relative to the magnetic field.
    void runFollowMagnetometer();

    // Attitude control relative to the Sun.
    void runFollowSun();

    // Demagnetize the magnetorquers.
    void runDemagnetize();

    // Rotate using the magnetorquers.
    void runMaximumMagneticTorque();

    // Set the rotational speed of the wheel.
    void runSetWheelSpeed();
};


extern ESATADCS ADCS;

#endif
