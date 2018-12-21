/*
 * Copyright (C) 2017 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_WheelPIDController_h
#define ESAT_WheelPIDController_h

#include <Arduino.h>

// Wheel control loop, PID variant.
// Use the public instance ESAT_WheelPIDController.
//
// This is a simple PID control loop.
class ESAT_WheelPIDControllerClass
{
  public:
    // Derivative gain of the PID control algorithm.
    // Expressed in seconds.
    float derivativeGain;

    // Integral gain of the PID control algorithm.
    // Expressed in 1 / second.
    float integralGain;

    // Proportional gain of the PID control algorithm.
    // Dimensionless.
    float proportionalGain;

    // Start the control loop.
    // Set the default gains.
    void begin();

    // Perform one iteration of the control loop.
    void loop(word targetSpeed);

    // Return the latest target speed of the wheel in revolutions per
    // minute.
    word readTargetSpeed();

    // Reset the error integral.
    void resetErrorIntegral();

  private:
    // Default value of the derivative gain of the PID control
    // algorithm.  Expressed in seconds.
    static constexpr float DEFAULT_DERIVATIVE_GAIN = 0;

    // Default value of the integral gain of the PID control
    // algorithm.  Expressed in 1 / second.
    static constexpr float DEFAULT_INTEGRAL_GAIN = 0.3;

    // Default value of the proportional gain of the PID control
    // algorithm.  Dimensionless.
    static constexpr float DEFAULT_PROPORTIONAL_GAIN = 1.5;

    // Integral of the error term.
    float errorIntegral;

    // Wheel speed (in revolutions per minute) at the previous
    // iteration of the control loop.
    int previousError;

    // Target rotational speed of the wheel in revolutions per minute.
    word targetSpeed;
};

// Global instance of the wheel PID controller library.
extern ESAT_WheelPIDControllerClass ESAT_WheelPIDController;

#endif /* ESAT_WheelPIDController_h */
