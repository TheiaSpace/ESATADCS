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

#include "ESAT_ADCS-actuators/ESAT_Magnetorquer.h"

void ESAT_MagnetorquerClass::begin()
{
  configurePins();
  writeEnable(false);
  writeX(NEGATIVE);
  writeY(NEGATIVE);
}

void ESAT_MagnetorquerClass::configurePins()
{
  pinMode(PIN_ENABLE_X, OUTPUT);
  pinMode(PIN_ENABLE_Y, OUTPUT);
  pinMode(PIN_X_POLARITY, OUTPUT);
  pinMode(PIN_Y_POLARITY, OUTPUT);
}

boolean ESAT_MagnetorquerClass::readEnable() const
{
  return enable;
}

ESAT_MagnetorquerClass::Polarity ESAT_MagnetorquerClass::readX() const
{
  return xPolarity;
}

ESAT_MagnetorquerClass::Polarity ESAT_MagnetorquerClass::readY() const
{
  return yPolarity;
}

void ESAT_MagnetorquerClass::writeEnable(const boolean enableDriver)
{
  enable = enableDriver;
  if (enable)
  {
    digitalWrite(PIN_ENABLE_X, HIGH);
    digitalWrite(PIN_ENABLE_Y, HIGH);
  }
  else
  {
    digitalWrite(PIN_ENABLE_X, LOW);
    digitalWrite(PIN_ENABLE_Y, LOW);
  }
}

void ESAT_MagnetorquerClass::writeX(const ESAT_MagnetorquerClass::Polarity polarity)
{
  xPolarity = polarity;
  if (polarity == NEGATIVE)
  {
    digitalWrite(PIN_X_POLARITY, LOW);
  }
  else
  {
    digitalWrite(PIN_X_POLARITY, HIGH);
  }
}

void ESAT_MagnetorquerClass::writeY(const ESAT_MagnetorquerClass::Polarity polarity)
{
  yPolarity = polarity;
  if (polarity == NEGATIVE)
  {
    digitalWrite(PIN_Y_POLARITY, LOW);
  }
  else
  {
    digitalWrite(PIN_Y_POLARITY, HIGH);
  }
}

ESAT_MagnetorquerClass ESAT_Magnetorquer;