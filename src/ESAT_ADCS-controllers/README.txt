Copyright (C) 2017 Theia Space, Universidad Politécnica de Madrid

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


High-level actuator and attitude controllers go here.


# ESAT_AttitudePIDController

For pointing the satellite to a fixed direction.  It uses
ESAT_MagnetorquerController and ESAT_WheelPIDController to apply the
actuation needed to point the satellite.


# ESAT_MagnetorquerController

For applying a fixed-direction torque with the magnetorquers.


# ESAT_WheelPIDController

For rotating the wheel at a fixed speed.
