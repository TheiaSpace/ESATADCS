/*
 *       ESAT ADCS library.
 *       Author Daniel Calvo @ Theia
 ******
 * Angles are defined as follows
 *
 *             X-(180º)
 *           __________
 *          |     --> Y||
 * Y-(270º) |    |     || panel  Y+(90º)
 *          |    V X   ||
 *          |__________||
 *           ----------
 *             panel
 *             X+(0º)
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

#include "ESAT_ADCS.h"
#ifdef ARDUINO_ESAT_ADCS
#include <ESAT_I2CSlave.h>
#include <Wire.h>
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
#include <ESAT_OBCClock.h>
#include <USBSerial.h>
#endif /* ARDUINO_ESAT_OBC */
#include "ESAT_ADCS-actuators/ESAT_Magnetorquer.h"
#include "ESAT_ADCS-actuators/ESAT_Wheel.h"
#include "ESAT_ADCS-controllers/ESAT_AttitudePIDController.h"
#include "ESAT_ADCS-controllers/ESAT_WheelPIDController.h"
#include "ESAT_ADCS-measurements/ESAT_CoarseSunSensor.h"
#include "ESAT_ADCS-measurements/ESAT_Gyroscope.h"
#include "ESAT_ADCS-measurements/ESAT_Magnetometer.h"
#include "ESAT_ADCS-measurements/ESAT_Tachometer.h"
#include "ESAT_ADCS-run-modes/ESAT_StopActuatorsRunMode.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_AttitudeTelecommandHandler.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_DiagnosticsTelecommandHandler.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_MagnetorquerTelecommandHandler.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_StopActuatorsTelecommandHandler.h"
#include "ESAT_ADCS-telecommand-handlers/ESAT_WheelTelecommandHandler.h"
#include "ESAT_ADCS-telemetry-packets/ESAT_ADCSHousekeepingTelemetryPacket.h"


void ESAT_ADCSClass::addHousekeepingTelemetryPacket()
{
  addTelemetryPacket(ESAT_ADCSHousekeepingTelemetryPacket);
}

void ESAT_ADCSClass::addTelemetryPacket(ESAT_ADCSTelemetryPacket& telemetryPacket)
{
  if (numberOfTelemetryPackets >= MAXIMUM_NUMBER_OF_TELEMETRY_PACKETS)
  {
    return;
  }
  telemetryPackets[numberOfTelemetryPackets] = &telemetryPacket;
  numberOfTelemetryPackets = numberOfTelemetryPackets + 1;
}

ESAT_AttitudeStateVector ESAT_ADCSClass::attitudeStateVector()
{
  return currentAttitudeStateVector;
}

void ESAT_ADCSClass::begin()
{
  currentUpdateTime = millis();
  previousUpdateTime = currentUpdateTime;
  telemetryPacketSequenceCount = 0;
  ESAT_AttitudePIDController.begin();
  ESAT_Wheel.begin();
  ESAT_WheelPIDController.begin();
  ESAT_Gyroscope.begin(ESAT_Gyroscope.FULL_SCALE_2000_DEGREES_PER_SECOND);
  ESAT_Magnetometer.begin();
  ESAT_CoarseSunSensor.begin();
  ESAT_Tachometer.begin();
  ESAT_Magnetorquer.begin();
  setRunMode(ESAT_StopActuatorsRunMode);
  numberOfTelemetryPackets = 0;
  numberOfTelecommandHandlers = 0;
  registerTelecommandHandler(ESAT_AttitudeTelecommandHandler);
  registerTelecommandHandler(ESAT_DiagnosticsTelecommandHandler);
  registerTelecommandHandler(ESAT_WheelTelecommandHandler);
  registerTelecommandHandler(ESAT_MagnetorquerTelecommandHandler);
  registerTelecommandHandler(ESAT_StopActuatorsTelecommandHandler);
#ifdef ARDUINO_ESAT_ADCS
  ESAT_I2CSlave.begin(Wire,
                      i2cTelecommandPacketData,
                      MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH,
                      i2cTelemetryPacketData,
                      MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH);
#endif /* ARDUINO_ESAT_ADCS */
}

void ESAT_ADCSClass::disableUSBTelecommands()
{
  usbTelecommandsEnabled = false;
}

void ESAT_ADCSClass::disableUSBTelemetry()
{
  usbTelemetryEnabled = false;
}

void ESAT_ADCSClass::enableUSBTelecommands(byte buffer[],
                                           const unsigned long bufferLength)
{
#ifdef ARDUINO_ESAT_ADCS
  usbTelecommandDecoder = ESAT_KISSStream(Serial,
                                          buffer,
                                          bufferLength);
  usbTelecommandsEnabled = true;
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  usbTelecommandDecoder = ESAT_KISSStream(USB,
                                          buffer,
                                          bufferLength);
  usbTelecommandsEnabled = true;
#endif /* ARDUINO_ESAT_OBC */
}

void ESAT_ADCSClass::enableUSBTelemetry()
{
  usbTelemetryEnabled = true;
}

boolean ESAT_ADCSClass::fillTelemetryPacket(ESAT_CCSDSPacket& packet,
                                            const byte identifier)
{
  if (packet.capacity() < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return false;
  }
  for (int index = 0; index < numberOfTelemetryPackets; index++)
  {
    if (!telemetryPackets[index])
    {
      return false;
    }
    if (telemetryPackets[index]->packetIdentifier() == identifier)
    {
#ifdef ARDUINO_ESAT_ADCS
      packet.writeTelemetryHeaders(getApplicationProcessIdentifier(),
                                   telemetryPacketSequenceCount,
                                   ESAT_Timestamp(),
                                   MAJOR_VERSION_NUMBER,
                                   MINOR_VERSION_NUMBER,
                                   PATCH_VERSION_NUMBER,
                                   telemetryPackets[index]->packetIdentifier());
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
      packet.writeTelemetryHeaders(getApplicationProcessIdentifier(),
                                   telemetryPacketSequenceCount,
                                   ESAT_OBCClock.read(),
                                   MAJOR_VERSION_NUMBER,
                                   MINOR_VERSION_NUMBER,
                                   PATCH_VERSION_NUMBER,
                                   telemetryPackets[index]->packetIdentifier());
#endif /* ARDUINO_ESAT_OBC */
      telemetryPackets[index]->readUserData(packet);
      telemetryPacketSequenceCount = telemetryPacketSequenceCount + 1;
      return true;
    }
  }
  return false;
}

word ESAT_ADCSClass::getApplicationProcessIdentifier()
{
  return APPLICATION_PROCESS_IDENTIFIER;
}

void ESAT_ADCSClass::handleTelecommand(ESAT_CCSDSPacket& packet)
{
  packet.rewind();
  const ESAT_CCSDSPrimaryHeader primaryHeader = packet.readPrimaryHeader();
  if (primaryHeader.applicationProcessIdentifier
      != getApplicationProcessIdentifier())
  {
    return;
  }
  if (primaryHeader.packetType != primaryHeader.TELECOMMAND)
  {
    return;
  }
  if (primaryHeader.packetDataLength < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return;
  }
  for (int i = 0; i < numberOfTelecommandHandlers; i++)
  {
    const boolean handled = telecommandHandlers[i]->handleTelecommand(packet);
    if (handled)
    {
      return;
    }
  }
}

float ESAT_ADCSClass::period()
{
  return (currentUpdateTime - previousUpdateTime) / 1000.;
}

void ESAT_ADCSClass::readSensors()
{
  currentAttitudeStateVector.wheelSpeed = ESAT_Tachometer.read();
  const boolean enableMagnetorquerDriver = ESAT_Magnetorquer.readEnable();
  ESAT_Magnetorquer.writeEnable(false);
  delay(20);
  ESAT_Gyroscope.error = false;
  currentAttitudeStateVector.rotationalSpeed = ESAT_Gyroscope.read(3);
  ESAT_Magnetometer.error = false;
  currentAttitudeStateVector.magneticAngle = ESAT_Magnetometer.read();
  ESAT_Magnetorquer.writeEnable(enableMagnetorquerDriver);
  currentAttitudeStateVector.sunAngle = ESAT_CoarseSunSensor.readSunAngle();
}

boolean ESAT_ADCSClass::readTelecommand(ESAT_CCSDSPacket& packet)
{
  (void) packet;
  packet.flush();
  if (packet.capacity() < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return false;
  }
  boolean pendingTelecommand = false;
#ifdef ARDUINO_ESAT_ADCS
  pendingTelecommand = ESAT_I2CSlave.readPacket(packet);
#endif /* ARDUINO_ESAT_ADCS */
  if (!pendingTelecommand && usbTelecommandsEnabled)
  {
    pendingTelecommand = readTelecommandFromUSB(packet);
  }
  if (!pendingTelecommand)
  {
    return false;
  }
  const ESAT_CCSDSPrimaryHeader primaryHeader = packet.readPrimaryHeader();
  if (primaryHeader.packetType != primaryHeader.TELECOMMAND)
  {
    return false;
  }
  if (primaryHeader.applicationProcessIdentifier
      != getApplicationProcessIdentifier())
  {
    return false;
  }
  if (primaryHeader.packetDataLength < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return false;
  }
  return true;
}

boolean ESAT_ADCSClass::readTelecommandFromUSB(ESAT_CCSDSPacket& packet)
{
  const boolean gotFrame = usbTelecommandDecoder.receiveFrame();
  if (!gotFrame)
  {
    return false;
  }
  return packet.readFrom(usbTelecommandDecoder);
}

boolean ESAT_ADCSClass::readTelemetry(ESAT_CCSDSPacket& packet)
{
  const int identifier = pendingTelemetry.readNext();
  if (identifier < 0)
  {
    return false;
  }
  const boolean gotPacket = fillTelemetryPacket(packet, byte(identifier));
  pendingTelemetry.clear(byte(identifier));
  return gotPacket;
}

void ESAT_ADCSClass::registerTelecommandHandler(ESAT_ADCSTelecommandHandler& telecommandHandler)
{
  if (numberOfTelecommandHandlers == MAXIMUM_NUMBER_OF_TELECOMMAND_HANDLERS)
  {
    return;
  }
  telecommandHandlers[numberOfTelecommandHandlers] = &telecommandHandler;
  numberOfTelecommandHandlers = numberOfTelecommandHandlers + 1;
}

void ESAT_ADCSClass::run()
{
  runMode->run();
}

byte ESAT_ADCSClass::runModeIdentifier()
{
  return runMode->identifier();
}

void ESAT_ADCSClass::setRunMode(ESAT_ADCSRunMode& newRunMode)
{
  runMode = &newRunMode;
}

boolean ESAT_ADCSClass::telemetryAvailable()
{
  if (pendingTelemetry.readNext() != pendingTelemetry.NO_ACTIVE_FLAGS)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void ESAT_ADCSClass::update()
{
  numberOfTelemetryPackets = 0;
  updatePeriod();
  readSensors();
  run();
  addHousekeepingTelemetryPacket();
  updatePendingTelemetryLists();
}

#ifdef ARDUINO_ESAT_ADCS
void ESAT_ADCSClass::respondToI2CRequests()
{
  const int requestedPacket = ESAT_I2CSlave.requestedPacket();
  switch (requestedPacket)
  {
    case ESAT_I2CSlave.NO_PACKET_REQUESTED:
      break;
    case ESAT_I2CSlave.NEXT_TELEMETRY_PACKET_REQUESTED:
      respondToNextPacketTelemetryRequest();
      break;
    case ESAT_I2CSlave.NEXT_TELECOMMAND_PACKET_REQUESTED:
      respondToNextPacketTelecommandRequest();
      break;
    default:
      respondToNamedPacketTelemetryRequest(byte(requestedPacket));
      break;
  }
}
#endif /* ARDUINO_ESAT_ADCS */

#ifdef ARDUINO_ESAT_ADCS
void ESAT_ADCSClass::respondToNamedPacketTelemetryRequest(const byte identifier)
{
  if (pendingTelemetry.read(identifier))
  {
    byte packetData[MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH];
    ESAT_CCSDSPacket packet(packetData, MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH);
    const boolean gotPacket = fillTelemetryPacket(packet, identifier);
    if (gotPacket)
    {
      ESAT_I2CSlave.writePacket(packet);
    }
    else
    {
      ESAT_I2CSlave.rejectPacket();
    }
  }
  else
  {
    ESAT_I2CSlave.rejectPacket();
  }
}
#endif /* ARDUINO_ESAT_ADCS */

#ifdef ARDUINO_ESAT_ADCS
void ESAT_ADCSClass::respondToNextPacketTelecommandRequest()
{
  ESAT_I2CSlave.rejectPacket();
}
#endif /* ARDUINO_ESAT_ADCS */

#ifdef ARDUINO_ESAT_ADCS
void ESAT_ADCSClass::respondToNextPacketTelemetryRequest()
{
  if (ESAT_I2CSlave.telemetryQueueResetReceived())
  {
    i2cPendingTelemetry = pendingTelemetry;
  }
  const int identifier = i2cPendingTelemetry.readNext();
  if (identifier >= 0)
  {
    byte packetData[MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH];
    ESAT_CCSDSPacket packet(packetData, MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH);
    const boolean gotPacket = fillTelemetryPacket(packet, identifier);
    if (gotPacket)
    {
      ESAT_I2CSlave.writePacket(packet);
    }
    i2cPendingTelemetry.clear(byte(identifier));
  }
  else
  {
    ESAT_I2CSlave.rejectPacket();
  }
}
#endif /* ARDUINO_ESAT_ADCS */

void ESAT_ADCSClass::updatePendingTelemetryLists()
{
  pendingTelemetry.clearAll();
  for (int index = 0; index < numberOfTelemetryPackets; index++)
  {
    const byte packetIdentifier = telemetryPackets[index]->packetIdentifier();
    pendingTelemetry.set(packetIdentifier);
  }
}

void ESAT_ADCSClass::updatePeriod()
{
  previousUpdateTime = currentUpdateTime;
  currentUpdateTime = millis();
}

void ESAT_ADCSClass::writeTelemetry(ESAT_CCSDSPacket& packet)
{
#ifdef ARDUINO_ESAT_ADCS
  if (usbTelemetryEnabled)
  {
    packet.rewind();
    const unsigned long encoderBufferLength =
      ESAT_KISSStream::frameLength(packet.length());
    byte encoderBuffer[encoderBufferLength];
    ESAT_KISSStream encoder(Serial, encoderBuffer, encoderBufferLength);
    (void) encoder.beginFrame();
    (void) packet.writeTo(encoder);
    (void) encoder.endFrame();
  }
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  if (usbTelemetryEnabled)
  {
    packet.rewind();
    const unsigned long encoderBufferLength =
      ESAT_KISSStream::frameLength(packet.length());
    byte encoderBuffer[encoderBufferLength];
    ESAT_KISSStream encoder(USB, encoderBuffer, encoderBufferLength);
    (void) encoder.beginFrame();
    (void) packet.writeTo(encoder);
    (void) encoder.endFrame();
  }
#endif /* ARDUINO_ESAT_OBC */
}

ESAT_ADCSClass ESAT_ADCS;
