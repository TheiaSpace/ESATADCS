/*
 * Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ESAT_ADCS-actuators/ESAT_ADCSLED.h"

void ESAT_ADCSLEDClass::begin()
{
#ifdef ARDUINO_ESAT_ADCS
  pinMode(LED_CONTROL_LINE, OUTPUT);
  write(0);
#endif /* ARDUINO_ESAT_ADCS */
}

void ESAT_ADCSLEDClass::write(const float brightness)
{
#ifdef ARDUINO_ESAT_OBC
  (void) brightness; // Unused.
#endif /* ARDUINO_ESAT_OBC */
#ifdef ARDUINO_ESAT_ADCS
  if (brightness < 0)
  {
    write(0);
  }
  else if (brightness > 100)
  {
    write(100);
  }
  else
  {
    const byte dutyCycle = map(brightness, 0, 100, 0, 255);
    analogWrite(LED_CONTROL_LINE, dutyCycle);
  }
#endif /* ARDUINO_ESAT_ADCS */
}

ESAT_ADCSLEDClass ESAT_ADCSLED;
