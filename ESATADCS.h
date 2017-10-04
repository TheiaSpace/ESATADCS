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
#include <Energia.h
#include <ESATCCSDSPacket.h>

class ESATADCS
{
  public:
    // True if the inertial measurement unit is alive.
    boolean inertialMeasurementUnitAlive;

    // Get all ADCS subsystems ready.
    void begin();

    // Handle a telecommand.
    void handleCommand(ESATCCSDSPacket& packet);

    // Fill a packet with the next ADCS telemetry packet available.
    void readTelemetry(ESATCCSDSPacket& packet);

    // Return true if there is a new telemetry packet available.
    boolean telemetryAvailable();

    void update();

  private:
    // Command codes.
    enum CommandCode
    {
      FOLLOW_MAGNETOMETER_COMMAND = 0x00,
      FOLLOW_SUN_COMMAND = 0x01,
      ATTITUDE_CONTROLLER_SET_PROPORTIONAL_GAIN_COMMAND = 0x10,
      ATTITUDE_CONTROLLER_SET_INTEGRAL_GAIN_COMMAND = 0x11,
      ATTITUDE_CONTROLLER_SET_DERIVATIVE_GAIN_COMMAND = 0x12,
      ATTITUDE_CONTROLLER_USE_GYROSCOPE_COMMAND = 0x13,
      ATTITUDE_CONTROLLER_USE_WHEEL_OR_MAGNETORQUER_COMMAND = 0x14,
      WHEEL_SET_DUTY_CYCLE_COMMAND = 0x20,
      WHEEL_SET_SPEED_COMMAND = 0x21,
      WHEEL_CONTROLLER_SET_PROPORTIONAL_GAIN_COMMAND = 0x30,
      WHEEL_CONTROLLER_SET_INTEGRAL_GAIN_COMMAND = 0x31,
      WHEEL_CONTROLLER_SET_DERIVATIVE_GAIN_COMMAND = 0x32,
      MAGNETORQUER_ENABLE_COMMAND = 0x40,
      MAGNETORQUER_SET_X_POLARITY_COMMAND = 0x41,
      MAGNETORQUER_SET_Y_POLARITY_COMMAND = 0x42,
      MAGNETORQUER_APPLY_MAXIMUM_TORQUE_COMMAND = 0x43,
      MAGNETORQUER_DEMAGNETIZE_COMMAND = 0x44,
      REST_COMMAND = 0xFF;
    };

    // Telemetry packet identifiers.
    enum TelemetryPacketIdentifier
    {
      HOUSEKEEPING = 0,
    };

    // Run codes.
    enum RunCode
    {
      FOLLOW_MAGNETOMETER = 0x00,
      FOLLOW_SUN = 0x01,
      WHEEL_SET_DUTY_CYCLE = 0x20,
      WHEEL_SET_SPEED = 0x21,
      MAGNETORQUER_ENABLE = 0x40,
      MAGNETORQUER_SET_X_POLARITY = 0x41,
      MAGNETORQUER_SET_Y_POLARITY = 0x42,
      MAGNETORQUER_APPLY_MAXIMUM_TORQUE = 0x43,
      MAGNETORQUER_DEMAGNETIZE = 0x44,
      REST = 0xFF,
    };

    // Minimum command packet length:
    // - Primary header (6 bytes).
    // - Secondary header (4 bytes).
    const byte MINIMUM_COMMAND_PACKET_LENGTH = 10;

    float attitudeDerivativeGain;
    float attitudeErrorIntegral;
    float attitudeIntegralGain;
    float attitudeProportionalGain;
    word demagnetizationIterations;
    boolean enableMagnetorquerDriver;
    int magneticAngle;
    int magnetorquerXPolarity;
    int magnetorquerYPolarity;
    boolean newTelemetryPacket;
    int oldAttitudeError;
    int oldWheelSpeedError;
    int rotationalSpeed;
    enum RunCode runCode;
    int sunAngle;
    int targetAttitude;
    boolean targetMagnetorquerDirection;
    int targetWheelSpeed;
    word telemetrySequenceCount;
    boolean useGyroscope;
    boolean useWheel;
    float wheelDerivativeGain;
    float wheelIntegralGain;
    float wheelProportionalGain;
    byte wheelDutyCycle;
    unsigned int wheelSpeed;
    float wheelSpeedErrorIntegral;

    // Light blink sequence performed at startup.
    void blinkSequence();

    // Commands.
    void handleFollowMagnetometerCommand(ESATCCSDSPacket& packet);
    void handleFollowSunCommand(ESATCCSDSPacket& packet);
    void handleAttitudeControllerSetProportionalGainCommand(ESATCCSDSPacket& packet);
    void handleAttitudeControllerSetIntegralGainCommand(ESATCCSDSPacket& packet);
    void handleAttitudeControllerSetDerivativeGainCommand(ESATCCSDSPacket& packet);
    void handleAttitudeControllerUseGyroscopeCommand(ESATCCSDSPacket& packet);
    void handleAttitudeControllerUseWheelOrMagnetorquerCommand(ESATCCSDSPacket& packet);
    void handleWheelSetDutyCycle(ESATCCSDSPacket& packet);
    void handleWheelSetSpeedCommand(ESATCCSDSPacket& packet);
    void handleWheelControllerSetProportionalGainCommand(ESATCCSDSPacket& packet);
    void handleWheelControllerSetIntegralGainCommand(ESATCCSDSPacket& packet);
    void handleWheelControllerSetDerivativeGainCommand(ESATCCSDSPacket& packet);
    void handleMagnetorquerEnable(ESATCCSDSPacket& packet);
    void handleMagnetorquerSetXPolarityCommand(ESATCCSDSPacket& packet);
    void handleMagnetorquerSetYPolarityCommand(ESATCCSDSPacket& packet);
    void handleMagnetorquerApplyMaximumTorqueCommand(ESATCCSDSPacket& packet);
    void handleMagnetorquerDemagnetizeCommand(ESATCCSDSPacket& packet);
    void handleRestCommand(ESATCCSDPacket& packet);

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

};


extern ESATADCS ADCS;

#endif
