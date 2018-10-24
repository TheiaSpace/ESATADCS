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

#include "ESAT_ADCS-actuators/ESAT_Magnetorquer.h"

void ESAT_MagnetorquerClass::begin()
{
  configurePins();
  writeEnable(false);
  writeX(NEGATIVE);
  writeY(NEGATIVE);
  invertXPolarity(false);
  invertYPolarity(false);
  swapAxes(false);
}

void ESAT_MagnetorquerClass::configurePins()
{
#ifdef ARDUINO_ESAT_ADCS
  pinMode(MTQXPLUS, OUTPUT);
  pinMode(MTQYPLUS, OUTPUT);
  pinMode(MTQXMINUS, OUTPUT);
  pinMode(MTQYMINUS, OUTPUT);
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  pinMode(ENMTQX, OUTPUT);
  pinMode(ENMTQY, OUTPUT);
  pinMode(MTQX, OUTPUT);
  pinMode(MTQY, OUTPUT);
#endif /* ARDUINO_ESAT_OBC */
}

int ESAT_MagnetorquerClass::invertLevel(const int level) const
{
  if (level == LOW)
  {
    return HIGH;
  }
  else
  {
    return LOW;
  }
}

void ESAT_MagnetorquerClass::invertXPolarity(const boolean invert)
{
  invertXPolarityLevel = invert;
}

void ESAT_MagnetorquerClass::invertYPolarity(const boolean invert)
{
  invertYPolarityLevel = invert;
}

#ifdef ARDUINO_ESAT_OBC
int ESAT_MagnetorquerClass::pinXPolarity() const
{
  if (swapPolarityPins)
  {
    return MTQY;
  }
  else
  {
    return MTQX;
  }
}
#endif /* ARDUINO_ESAT_OBC */

#ifdef ARDUINO_ESAT_ADCS
int ESAT_MagnetorquerClass::pinXPolarityMinus() const
{
  if (swapPolarityPins)
  {
    return MTQYMINUS;
  }
  else
  {
    return MTQXMINUS;
  }
}
#endif /* ARDUINO_ESAT_ADCS */

#ifdef ARDUINO_ESAT_ADCS
int ESAT_MagnetorquerClass::pinXPolarityPlus() const
{
  if (swapPolarityPins)
  {
    return MTQYPLUS;
  }
  else
  {
    return MTQXPLUS;
  }
}
#endif /* ARDUINO_ESAT_ADCS */

#ifdef ARDUINO_ESAT_OBC
int ESAT_MagnetorquerClass::pinYPolarity() const
{
  if (swapPolarityPins)
  {
    return MTQX;
  }
  else
  {
    return MTQY;
  }
}
#endif /* ARDUINO_ESAT_OBC */

#ifdef ARDUINO_ESAT_ADCS
int ESAT_MagnetorquerClass::pinYPolarityMinus() const
{
  if (swapPolarityPins)
  {
    return MTQXMINUS;
  }
  else
  {
    return MTQYMINUS;
  }
}
#endif /* ARDUINO_ESAT_ADCS */

#ifdef ARDUINO_ESAT_ADCS
int ESAT_MagnetorquerClass::pinYPolarityPlus() const
{
  if (swapPolarityPins)
  {
    return MTQXPLUS;
  }
  else
  {
    return MTQYPLUS;
  }
}
#endif /* ARDUINO_ESAT_ADCS */

int ESAT_MagnetorquerClass::polarityLevel(const ESAT_MagnetorquerClass::Polarity polarity) const
{
  if (polarity == NEGATIVE)
  {
    return LOW;
  }
  else
  {
    return HIGH;
  }
}

int ESAT_MagnetorquerClass::polarityXLevel() const
{
  const int level = polarityLevel(xPolarity);
  if (invertXPolarityLevel)
  {
    return invertLevel(level);
  }
  else
  {
    return level;
  }
}

int ESAT_MagnetorquerClass::polarityYLevel() const
{
  const int level = polarityLevel(yPolarity);
  if (invertYPolarityLevel)
  {
    return invertLevel(level);
  }
  else
  {
    return level;
  }
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

void ESAT_MagnetorquerClass::swapAxes(const boolean swap)
{
  swapPolarityPins = swap;
}

void ESAT_MagnetorquerClass::writeEnable(const boolean enableDriver)
{
  enable = enableDriver;
#ifdef ARDUINO_ESAT_ADCS
  if (enable)
  {
    writeX(readX());
    writeY(readY());
  }
  else
  {
    digitalWrite(MTQXPLUS, LOW);
    digitalWrite(MTQYPLUS, LOW);
    digitalWrite(MTQXMINUS, LOW);
    digitalWrite(MTQYMINUS, LOW);
  }
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  if (enable)
  {
    digitalWrite(ENMTQX, HIGH);
    digitalWrite(ENMTQY, HIGH);
  }
  else
  {
    digitalWrite(ENMTQX, LOW);
    digitalWrite(ENMTQY, LOW);
  }
#endif /* ARDUINO_ESAT_OBC */
}

void ESAT_MagnetorquerClass::writeX(const ESAT_MagnetorquerClass::Polarity polarity)
{
  xPolarity = polarity;
#ifdef ARDUINO_ESAT_ADCS
  if (enable)
  {
    const int pinPlus = pinXPolarityPlus();
    const int pinMinus = pinXPolarityMinus();
    const int levelPlus = polarityXLevel();
    const int levelMinus = invertLevel(levelPlus);
    digitalWrite(pinPlus, levelPlus);
    digitalWrite(pinMinus, levelMinus);
  }
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  const int pin = pinXPolarity();
  const int level = polarityXLevel();
  digitalWrite(pin, level);
#endif /* ARDUINO_ESAT_OBC */
}

void ESAT_MagnetorquerClass::writeY(const ESAT_MagnetorquerClass::Polarity polarity)
{
  yPolarity = polarity;
#ifdef ARDUINO_ESAT_ADCS
  if (enable)
  {
    const int pinPlus = pinYPolarityPlus();
    const int pinMinus = pinYPolarityMinus();
    const int levelPlus = polarityYLevel();
    const int levelMinus = invertLevel(levelPlus);
    digitalWrite(pinPlus, levelPlus);
    digitalWrite(pinMinus, levelMinus);
  }
#endif /* ARDUINO_ESAT_ADCS */
#ifdef ARDUINO_ESAT_OBC
  const int pin = pinYPolarity();
  const int level = polarityYLevel();
  digitalWrite(pin, level);
#endif /* ARDUINO_ESAT_OBC */
}

ESAT_MagnetorquerClass ESAT_Magnetorquer;
