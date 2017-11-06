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

#ifndef ESAT_ADCS_h
#define ESAT_ADCS_h
#include <Arduino.h>
#include <ESAT_CCSDSPacket.h>
#include "ESAT_AttitudeStateVector.h"

// Attitude determination and control subsystem library.
// Use the global instance ESAT_ADCS.
class ESAT_ADCSClass
{
  public:
    // True if the inertial measurement unit is alive.
    boolean inertialMeasurementUnitAlive;

    // Get all ADCS subsystems ready.
    // Set the control cycle period (in milliseconds).
    void begin(word period);

    // Handle a telecommand.
    void handleTelecommand(ESAT_CCSDSPacket& packet);

    // Return the unique identifier of the ADCS.
    word getApplicationProcessIdentifier();

    // Fill a packet with the next ADCS telemetry packet available.
    // Return true if the operation was successful;
    // otherwise return false.
    boolean readTelemetry(ESAT_CCSDSPacket& packet);

    // Return true if there is a new telemetry packet available.
    boolean telemetryAvailable();

    void update();

  private:
    // Actuators used for attitude control.
    enum Actuator
    {
      MAGNETORQUER = 0,
      WHEEL = 1,
    };

    // Command codes.
    enum CommandCode
    {
      FOLLOW_MAGNETIC_ANGLE_COMMAND = 0x00,
      FOLLOW_SUN_ANGLE_COMMAND = 0x01,
      ATTITUDE_CONTROLLER_SET_PROPORTIONAL_GAIN_COMMAND = 0x10,
      ATTITUDE_CONTROLLER_SET_INTEGRAL_GAIN_COMMAND = 0x11,
      ATTITUDE_CONTROLLER_SET_DERIVATIVE_GAIN_COMMAND = 0x12,
      ATTITUDE_CONTROLLER_USE_GYROSCOPE_COMMAND = 0x13,
      ATTITUDE_CONTROLLER_SET_ACTUATOR_COMMAND = 0x14,
      ATTITUDE_CONTROLLER_SET_DEADBAND_COMMAND = 0x15,
      ATTITUDE_CONTROLLER_SET_DETUMBLING_THRESHOLD_COMMAND = 0x16,
      WHEEL_SET_DUTY_CYCLE_COMMAND = 0x20,
      WHEEL_SET_SPEED_COMMAND = 0x21,
      WHEEL_CONTROLLER_SET_PROPORTIONAL_GAIN_COMMAND = 0x30,
      WHEEL_CONTROLLER_SET_INTEGRAL_GAIN_COMMAND = 0x31,
      WHEEL_CONTROLLER_SET_DERIVATIVE_GAIN_COMMAND = 0x32,
      WHEEL_CONTROLLER_RESET_SPEED_ERROR_INTEGRAL = 0x33,
      MAGNETORQUER_ENABLE_COMMAND = 0x40,
      MAGNETORQUER_SET_X_POLARITY_COMMAND = 0x41,
      MAGNETORQUER_SET_Y_POLARITY_COMMAND = 0x42,
      MAGNETORQUER_APPLY_MAXIMUM_TORQUE_COMMAND = 0x43,
      MAGNETORQUER_DEMAGNETIZE_COMMAND = 0x44,
      STOP_ACTUATORS_COMMAND = 0xFF,
    };

    // Telemetry packet identifiers.
    enum TelemetryPacketIdentifier
    {
      HOUSEKEEPING = 0,
    };

    // Run codes.
    enum RunCode
    {
      FOLLOW_MAGNETIC_ANGLE = 0x00,
      FOLLOW_SUN_ANGLE = 0x01,
      WHEEL_SET_DUTY_CYCLE = 0x20,
      WHEEL_SET_SPEED = 0x21,
      MAGNETORQUER_ENABLE = 0x40,
      MAGNETORQUER_SET_X_POLARITY = 0x41,
      MAGNETORQUER_SET_Y_POLARITY = 0x42,
      MAGNETORQUER_APPLY_MAXIMUM_TORQUE = 0x43,
      MAGNETORQUER_DEMAGNETIZE = 0x44,
      STOP_ACTUATORS = 0xFF,
    };

    // Unique identifier of the subsystem.
    static const byte APPLICATION_PROCESS_IDENTIFIER = 2;

    // Version numbers.
    static const byte MAJOR_VERSION_NUMBER = 3;
    static const byte MINOR_VERSION_NUMBER = 0;
    static const byte PATCH_VERSION_NUMBER = 0;

    // Minimum command payload data length in bytes:
    // - Year (2 byte).
    // - Month (1 byte).
    // - Day (1 byte).
    // - Hours (1 byte).
    // - Minutes (1 byte).
    // - Seconds (1 byte).
    // - Major version number (1 byte).
    // - Minor version number (1 byte).
    // - Patch version number (1 byte).
    // - Command code (1 byte).
    static const byte MINIMUM_COMMAND_PAYLOAD_DATA_LENGTH = 11;

    // Size of the housekeeping telemetry packet in bytes:
    // - Primary header (6 bytes).
    // - Secondary header (4 bytes).
    // - Mode of operation (1 byte).
    // - Target attitude (2 bytes).
    // - Magnetic angle (2 bytes).
    // - Sun angle (2 bytes).
    // - Rotational speed (2 bytes).
    // - Attitude proportional gain (4 bytes).
    // - Attitude integral gain (4 bytes).
    // - Attitude derivative gain (4 bytes).
    // - Gyroscope usage for attitude error derivative determination (1 byte).
    // - Wheel or magnetorquer usage for attitude control (1 byte).
    // - Wheel duty cycle (4 byte).
    // - Wheel speed (2 bytes).
    // - Wheel proportional gain (4 bytes).
    // - Wheel integral gain (4 bytes).
    // - Wheel derivative gain (4 bytes).
    // - Magnetorquer driver state (1 byte).
    // - Magnetorquer X polarity (1 byte).
    // - Magnetorquer Y polarity (1 byte).
    // - Gyroscope error (1 byte).
    // - Magnetometer error (1 byte).
    static const byte HOUSEKEEPING_TELEMETRY_PACKET_LENGTH = 56;

    boolean newTelemetryPacket;
    enum RunCode runCode;
    ESAT_AttitudeStateVector attitudeStateVector;
    word telemetryPacketSequenceCount;

    // Commands.
    void handleFollowMagneticAngleCommand(ESAT_CCSDSPacket& packet);
    void handleFollowSunAngleCommand(ESAT_CCSDSPacket& packet);
    void handleAttitudeControllerSetProportionalGainCommand(ESAT_CCSDSPacket& packet);
    void handleAttitudeControllerSetIntegralGainCommand(ESAT_CCSDSPacket& packet);
    void handleAttitudeControllerSetDerivativeGainCommand(ESAT_CCSDSPacket& packet);
    void handleAttitudeControllerUseGyroscopeCommand(ESAT_CCSDSPacket& packet);
    void handleAttitudeControllerSetActuatorCommand(ESAT_CCSDSPacket& packet);
    void handleAttitudeControllerSetDeadbandCommand(ESAT_CCSDSPacket& packet);
    void handleAttitudeControllerSetDetumblingThresholdCommand(ESAT_CCSDSPacket& packet);
    void handleWheelSetDutyCycleCommand(ESAT_CCSDSPacket& packet);
    void handleWheelSetSpeedCommand(ESAT_CCSDSPacket& packet);
    void handleWheelControllerSetProportionalGainCommand(ESAT_CCSDSPacket& packet);
    void handleWheelControllerSetIntegralGainCommand(ESAT_CCSDSPacket& packet);
    void handleWheelControllerSetDerivativeGainCommand(ESAT_CCSDSPacket& packet);
    void handleWheelControllerResetSpeedErrorIntegral(ESAT_CCSDSPacket& packet);
    void handleMagnetorquerEnableCommand(ESAT_CCSDSPacket& packet);
    void handleMagnetorquerSetXPolarityCommand(ESAT_CCSDSPacket& packet);
    void handleMagnetorquerSetYPolarityCommand(ESAT_CCSDSPacket& packet);
    void handleMagnetorquerApplyMaximumTorqueCommand(ESAT_CCSDSPacket& packet);
    void handleMagnetorquerDemagnetizeCommand(ESAT_CCSDSPacket& packet);
    void handleStopActuatorsCommand(ESAT_CCSDSPacket& packet);

    // Read the sensors needed for attitude determination and control.
    void readSensors();

    // Actuate according to the current run code.
    void run();

    // Attitude control loop.
    void runAttitudeControlLoop(int currentAttitude);

    // Attitude control relative to the magnetic field.
    void runFollowMagneticAngle();

    // Attitude control relative to the Sun.
    void runFollowSunAngle();

    // Set the duty cycle of the wheel.
    void runWheelSetDutyCycle();

    // Set the rotational speed of the wheel.
    void runWheelSetSpeed();

    // Enable the magnetorquers.
    void runMagnetorquerEnable();

    // Set the polarity of the X magnetorquer.
    void runMagnetorquerSetXPolarity();

    // Set the polarity of the Y magnetorquer.
    void runMagnetorquerSetYPolarity();

    // Rotate using the magnetorquers.
    void runMagnetorquerApplyMaximumTorque();

    // Demagnetize the magnetorquers.
    void runMagnetorquerDemagnetize();

    // Stop actuators.
    void runStopActuators();
};

// Global instance of the ADCS library.
extern ESAT_ADCSClass ESAT_ADCS;

#endif /* ESAT_ADCS_h */
