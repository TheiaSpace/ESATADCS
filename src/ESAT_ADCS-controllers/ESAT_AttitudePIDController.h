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

#ifndef ESAT_AttitudePIDController_h
#define ESAT_AttitudePIDController_h

#include <Arduino.h>

// Wheel control loop, PID variant.
// Use the public instance ESAT_WheelPIDController.
//
// This is a modified PID control loop.
class ESAT_AttitudePIDControllerClass
{
  public:
  // Actuators used for attitude control.
    enum Actuator
    {
      MAGNETORQUER = 0,
      WHEEL = 1,
    };

    // Use this actuator for attitude control.
    Actuator actuator;

    // Derivative gain of the PID control algorithm.
    // Expressed in seconds.
    float derivativeGain;

    // Detumbling theshold (expressed in degrees per second): limit
    // control to the derivative correction when the error derivative
    // is larger than the detumbling threshold.
    word detumblingThreshold;

    // Error deadband (expressed in degrees): don't control when the
    // error is smaller than the error deadband and the error
    // derivative is smaller than the error derivative deadband.
    word errorDeadband;

    // Error derivative deadband (expressed in degrees per second):
    // don't control when the error is smaller than the error deadband
    // and the error derivative is smaller than the error derivative
    // deadband.
    word errorDerivativeDeadband;

    // When true, use the gyroscope for error derivative estimation;
    // when false, use numerical differentiation.
    boolean gyroscopeUsage;

    // Integral gain of the PID control algorithm.
    // Expressed in 1 / second.
    float integralGain;

    // Proportional gain of the PID control algorithm.
    // Dimensionless.
    float proportionalGain;

    // Target angle of a reference direction with respect to the +X
    // axis, measured counterclockwise.  This is the same as the
    // attitude of the +X axis relative to a reference direction,
    // measured clockwise.  From 0 degrees to 359 degrees.
    word targetAngle;

    // Start the control loop.
    // Set the default gains.
    // Set the default deadband.
    // Set the default detumbling threshold.
    void begin();

    // Perform one iteration of the control loop to try to compensate
    // for the angle error.
    void loop(word currentAngle);

    // Reset the error integral.
    void resetErrorIntegral();

  private:
    // Default actuator to use for attitude control.
    static const Actuator DEFAULT_ACTUATOR = MAGNETORQUER;

    // Default value of the derivative gain of the PID control
    // algorithm.  Expressed in seconds.
    static constexpr float DEFAULT_DERIVATIVE_GAIN = 22;

    // Default value of the detumbling theshold (expressed in degrees
    // per second): limit control to the derivative correction when
    // the error derivative is larger than the detumbling threshold.
    static const word DEFAULT_DETUMBLING_THRESHOLD = 40;

    // Default value of the error deadband (expressed in degrees):
    // don't control when the error is smaller than the error deadband
    // and the error derivative is smaller than the error derivative
    // deadband.
    static const word DEFAULT_ERROR_DEADBAND = 1;

    // Default value of the error derivative deadband (expressed in
    // degrees): don't control when the error is smaller than the
    // error deadband and the error derivative is smaller than the
    // error derivative deadband.
    static const word DEFAULT_ERROR_DERIVATIVE_DEADBAND = 2;

    // Default value of the gyroscopeUsage property.
    static const boolean DEFAULT_GYROSCOPE_USAGE_VALUE = true;

    // Default value of the integral gain of the PID control
    // algorithm.  Expressed in 1 / second.
    static constexpr float DEFAULT_INTEGRAL_GAIN = 0;

    // Default value of the proportional gain of the PID control
    // algorithm.  Dimensionless.
    static constexpr float DEFAULT_PROPORTIONAL_GAIN = 5.5;

    // Current actuation signal for compensating for the error signal.
    float actuation;

    // Error signal.
    int error;

    // Time derivative of the error signal.
    float errorDerivative;

    // Time integral of the error signal.
    float errorIntegral;

    // Angle error (in degrees) at the previous iteration of the
    // control loop.
    int previousError;

    // Return true if the error derivative is above the detumbling
    // threshold.  Otherwise return false.
    boolean aboveDetumblingThreshold();

    // Subtract two angles and wrap the result to be between -180
    // degrees and +180 degrees.
    int angleDifference(int minuend, int subtrahend);

    // Apply the desired actuation.
    void applyActuation();

    // Apply the desired actuation using the magnetorquers.
    void applyMagnetorquerActuation();

    // Apply the desired actuation using the wheel.
    void applyWheelActuation();

    // Return true if the following conditions are met simultaneously:
    // * The error is below the error deadband threshold.
    // * The error derivative is below the error derivative deadband threshold.
    // Return false otherwise.
    boolean belowDeadbandThreshold();

    // Compute the value of the actuation to compensate for the error
    // signal.
    float computeActuation();

    // Compute the error time derivative.
    float computeErrorDerivative();

    // Update the value of the actuation to compensate for the error
    // signal.
    void updateActuation();

    // Update the value of the error.
    void updateError(word currentAngle);

    // Update the value of the error derivative.
    void updateErrorDerivative();

    // Update the value of the error integral.
    void updateErrorIntegral();
};

// Global instance of the attitude PID controller library.
extern ESAT_AttitudePIDControllerClass ESAT_AttitudePIDController;

#endif /* ESAT_AttitudePIDController_h */
