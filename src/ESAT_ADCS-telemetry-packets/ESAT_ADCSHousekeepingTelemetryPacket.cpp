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

#include "ESAT_ADCS-telemetry-packets/ESAT_ADCSHousekeepingTelemetryPacket.h"
#include "ESAT_ADCS.h"
#include "ESAT_ADCS-actuators/ESAT_Magnetorquer.h"
#include "ESAT_ADCS-actuators/ESAT_Wheel.h"
#include "ESAT_ADCS-controllers/ESAT_AttitudePIDController.h"
#include "ESAT_ADCS-controllers/ESAT_WheelPIDController.h"
#include "ESAT_ADCS-measurements/ESAT_CoarseSunSensor.h"
#include "ESAT_ADCS-measurements/ESAT_Gyroscope.h"
#include "ESAT_ADCS-measurements/ESAT_Magnetometer.h"

byte ESAT_ADCSHousekeepingTelemetryPacketClass::packetIdentifier()
{
  return PACKET_IDENTIFIER;
}

void ESAT_ADCSHousekeepingTelemetryPacketClass::readUserData(ESAT_CCSDSPacket& packet)
{
  const ESAT_AttitudeStateVector attitudeStateVector =
    ESAT_ADCS.attitudeStateVector();
  packet.writeByte(ESAT_ADCS.runModeIdentifier());
  packet.writeWord(ESAT_AttitudePIDController.targetAngle);
  packet.writeWord(attitudeStateVector.magneticAngle);
  packet.writeWord(attitudeStateVector.sunAngle);
  packet.writeFloat(ESAT_CoarseSunSensor.readXPlus());
  packet.writeFloat(ESAT_CoarseSunSensor.readYPlus());
  packet.writeFloat(ESAT_CoarseSunSensor.readXMinus());
  packet.writeFloat(ESAT_CoarseSunSensor.readYMinus());
  packet.writeInt(attitudeStateVector.rotationalSpeed);
  packet.writeFloat(ESAT_AttitudePIDController.proportionalGain);
  packet.writeFloat(ESAT_AttitudePIDController.integralGain);
  packet.writeFloat(ESAT_AttitudePIDController.derivativeGain);
  packet.writeBoolean(ESAT_AttitudePIDController.gyroscopeUsage);
  packet.writeByte(ESAT_AttitudePIDController.actuator);
  packet.writeWord(ESAT_AttitudePIDController.errorDeadband);
  packet.writeWord(ESAT_AttitudePIDController.errorDerivativeDeadband);
  packet.writeWord(ESAT_AttitudePIDController.detumblingThreshold);
  packet.writeFloat(ESAT_Wheel.readDutyCycle());
  packet.writeWord(attitudeStateVector.wheelSpeed);
  packet.writeFloat(ESAT_WheelPIDController.proportionalGain);
  packet.writeFloat(ESAT_WheelPIDController.integralGain);
  packet.writeFloat(ESAT_WheelPIDController.derivativeGain);
  packet.writeByte(ESAT_Magnetorquer.readEnable());
  packet.writeByte(byte(ESAT_Magnetorquer.readX()));
  packet.writeByte(byte(ESAT_Magnetorquer.readY()));
  packet.writeBoolean(ESAT_Gyroscope.error);
  packet.writeBoolean(ESAT_Magnetometer.error);
}

ESAT_ADCSHousekeepingTelemetryPacketClass ESAT_ADCSHousekeepingTelemetryPacket;
