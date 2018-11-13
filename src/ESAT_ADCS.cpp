/*
 * Copyright (C) 2017, 2018 Theia Space, Universidad Politécnica de Madrid
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

void ESAT_ADCSClass::addTelemetryPacket(ESAT_ADCSTelemetryPacket& newTelemetryPacket)
{
  newTelemetryPacket.nextTelemetryPacket = telemetryPacket;
  telemetryPacket = &newTelemetryPacket;
#ifdef ARDUINO_ESAT_ADCS
  latestTelemetryPacket = &newTelemetryPacket;
#endif /* ARDUINO_ESAT_ADCS */
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
  telemetryPacket = nullptr;
#ifdef ARDUINO_ESAT_ADCS
  latestTelemetryPacket = nullptr;
  i2cTelemetryPacket = nullptr;
#endif /* ARDUINO_ESAT_ADCS */
  telecommandHandler = nullptr;
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

void ESAT_ADCSClass::clearTelemetryPacketList()
{
  telemetryPacket = nullptr;
#ifdef ARDUINO_ESAT_ADCS
  latestTelemetryPacket = nullptr;
#endif /* ARDUINO_ESAT_ADCS */
}

void ESAT_ADCSClass::disableUSBTelecommands()
{
  usbReader = ESAT_CCSDSPacketFromKISSFrameReader();
}

void ESAT_ADCSClass::disableUSBTelemetry()
{
  usbWriter = ESAT_CCSDSPacketToKISSFrameWriter();
}

void ESAT_ADCSClass::enableUSBTelecommands(byte buffer[],
                                           const unsigned long bufferLength)
{
  usbReader = ESAT_CCSDSPacketFromKISSFrameReader(Serial,
                                                  buffer,
                                                  bufferLength);
}

void ESAT_ADCSClass::enableUSBTelemetry()
{
  usbWriter = ESAT_CCSDSPacketToKISSFrameWriter(Serial);
}

boolean ESAT_ADCSClass::fillTelemetryPacket(ESAT_CCSDSPacket& packet,
                                            ESAT_ADCSTelemetryPacket& contents)
{
  packet.writeTelemetryHeaders(getApplicationProcessIdentifier(),
                               telemetryPacketSequenceCount,
                               clock.read(),
                               MAJOR_VERSION_NUMBER,
                               MINOR_VERSION_NUMBER,
                               PATCH_VERSION_NUMBER,
                               contents.packetIdentifier());
  contents.readUserData(packet);
  if (packet.triedToWriteBeyondCapacity())
  {
    return false;
  }
  else
  {
    telemetryPacketSequenceCount = telemetryPacketSequenceCount + 1;
    return true;
  }
}

ESAT_ADCSTelemetryPacket* ESAT_ADCSClass::findTelemetryPacket(const byte identifier)
{
  for (ESAT_ADCSTelemetryPacket* packet = telemetryPacket;
       packet != nullptr;
       packet = packet->nextTelemetryPacket)
  {
    if (packet->packetIdentifier() == identifier)
    {
      return packet;
    }
  }
  return nullptr;
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
  for (ESAT_ADCSTelecommandHandler* handler = telecommandHandler;
       handler != nullptr;
       handler = handler->nextTelecommandHandler)
  {
    const boolean handled = handler->handleTelecommand(packet);
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
  if (!pendingTelecommand)
  {
    pendingTelecommand = usbReader.read(packet);
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

boolean ESAT_ADCSClass::readTelemetry(ESAT_CCSDSPacket& packet)
{
  if (telemetryPacket == nullptr)
  {
    return false;
  }
  const boolean gotPacket = fillTelemetryPacket(packet,
                                                *telemetryPacket);
  telemetryPacket = telemetryPacket->nextTelemetryPacket;
  return gotPacket;
}

void ESAT_ADCSClass::registerTelecommandHandler(ESAT_ADCSTelecommandHandler& newTelecommandHandler)
{
  newTelecommandHandler.nextTelecommandHandler = telecommandHandler;
  telecommandHandler = &newTelecommandHandler;
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

void ESAT_ADCSClass::setTime(const ESAT_Timestamp timestamp)
{
  clock.write(timestamp);
}

boolean ESAT_ADCSClass::telemetryAvailable()
{
  if (telemetryPacket != nullptr)
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
  clearTelemetryPacketList();
  updatePeriod();
  readSensors();
  run();
  addHousekeepingTelemetryPacket();
}

void ESAT_ADCSClass::respondToI2CRequests()
{
  #ifdef ARDUINO_ESAT_ADCS
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
  #endif /* ARDUINO_ESAT_ADCS */
}

#ifdef ARDUINO_ESAT_ADCS
void ESAT_ADCSClass::respondToNamedPacketTelemetryRequest(const byte identifier)
{
  ESAT_ADCSTelemetryPacket* const matchingTelemetryPacket =
    findTelemetryPacket(identifier);
  if (matchingTelemetryPacket != nullptr)
  {
    byte packetData[MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH];
    ESAT_CCSDSPacket packet(packetData, MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH);
    const boolean gotPacket = fillTelemetryPacket(packet,
                                                  *matchingTelemetryPacket);
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
    i2cTelemetryPacket = latestTelemetryPacket;
  }
  if (i2cTelemetryPacket != nullptr)
  {
    byte packetData[MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH];
    ESAT_CCSDSPacket packet(packetData, MAXIMUM_TELEMETRY_PACKET_DATA_LENGTH);
    const boolean gotPacket = fillTelemetryPacket(packet,
                                                  *i2cTelemetryPacket);
    if (gotPacket)
    {
      ESAT_I2CSlave.writePacket(packet);
    }
    else
    {
      ESAT_I2CSlave.rejectPacket();
    }
    i2cTelemetryPacket = i2cTelemetryPacket->nextTelemetryPacket;
  }
  else
  {
    ESAT_I2CSlave.rejectPacket();
  }
}
#endif /* ARDUINO_ESAT_ADCS */

void ESAT_ADCSClass::updatePeriod()
{
  previousUpdateTime = currentUpdateTime;
  currentUpdateTime = millis();
}

void ESAT_ADCSClass::writeTelemetry(ESAT_CCSDSPacket& packet)
{
  (void) usbWriter.unbufferedWrite(packet);
}

ESAT_ADCSClass ESAT_ADCS;
