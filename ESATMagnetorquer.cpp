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

#include "ESATMagnetorquer.h"

void ESATMagnetorquer::begin()
{
  configurePins();
  writeEnable(false);
}

void ESATMagnetorquer::configurePins()
{
  pinMode(PIN_ENABLE_X, OUTPUT);
  pinMode(PIN_ENABLE_Y, OUTPUT);
  pinMode(PIN_X_POLARITY, OUTPUT);
  pinMode(PIN_Y_POLARITY, OUTPUT);
}

void ESATMagnetorquer::writeEnable(boolean enable)
{
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

void ESATMagnetorquer::writeX(ESATMagnetorquer::Polarity polarity)
{
  if (polarity == NEGATIVE)
  {
    digitalWrite(PIN_X_POLARITY, LOW);
  }
  else
  {
    digitalWrite(PIN_X_POLARITY, HIGH);
  }
}

void ESATMagnetorquer::writeY(ESATMagnetorquer::Polarity polarity)
{
  if (polarity == NEGATIVE)
  {
    digitalWrite(PIN_Y_POLARITY, LOW);
  }
  else
  {
    digitalWrite(PIN_Y_POLARITY, HIGH);
  }
}

ESATMagnetorquer Magnetorquer;
