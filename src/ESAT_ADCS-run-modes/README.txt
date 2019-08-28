Copyright (C) 2017, 2019 Theia Space, Universidad Politécnica de Madrid

This file is part of Theia Space's ESAT ADCS library.

Theia Space's ESAT ADCS library is free software: you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation, either
version 3 of the License, or (at your option) any later version.

Theia Space's ESAT ADCS library is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Theia Space's ESAT ADCS library.  If not, see
<http://www.gnu.org/licenses/>.


ADCS run modes abstract the modes of operation of the ADCS.


# ESAT_ADCSRunMode

Run modes must implement this interface, which exposes two methods:

  byte identifier();

  void run();

The identifier must be unique for each run mude.

The run() method performs whatever actions are relevant to the run
mode, like running the attitude control loop or simply switching on
and off actuators.

To change the run mode, use ESAT_ADCS.setRunMode()


# Run modes

  FOLLOW_MAGNETIC_TARGET: 0x00
  FOLLOW_SOLAR_TARGET: 0x01
  DETUMBLE: 0x02
  WHEEL_SET_DUTY_CYCLE: 0x20
  WHEEL_SET_SPEED: 0x21
  WHEEL_RESET_ELECTRONIC_SPEED_CONTROLLER: 0x22
  MAGNETORQUER_ENABLE: 0x30
  MAGNETORQUER_SET_X_POLARITY: 0x31
  MAGNETORQUER_SET_Y_POLARITY: 0x32
  MAGNETORQUER_APPLY_MAXIMUM_TORQUE: 0x33
  MAGNETORQUER_DEMAGNETIZE: 0x34
  STOP_ACTUATORS: 0xFF
