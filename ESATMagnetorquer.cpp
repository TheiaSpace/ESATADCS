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
  pinMode(pinEnableX, OUTPUT);
  pinMode(pinEnableY, OUTPUT);
  pinMode(pinX, OUTPUT);
  pinMode(pinY, OUTPUT);
}

void ESATMagnetorquer::writeEnable(boolean enable)
{
  digitalWrite(pinEnableX, enable ? HIGH : LOW);
  digitalWrite(pinEnableY, enable ? HIGH : LOW);
}

void ESATMagnetorquer::writeX(int polarity)
{
  digitalWrite(pinX, polarity);
}

void ESATMagnetorquer::writeY(int polarity)
{
  digitalWrite(pinY, polarity);
}

ESATMagnetorquer Magnetorquer;
