#ifndef __STATE_SPACE_METHODS__H__
#define __STATE_SPACE_METHODS__H__

// Calculates Voltage Needed To Reach Given Acceleration Target given the Current Speed and shape of Velocity Exponential Curve.
// @returns Relative Voltage double from -1 to 1. Multiply output by maximum voltage of system.

extern double calcVoltageSetpoint(double targetAcceleration, double decayExponent, double currentSpeed, double maxSpeed);

// Calculates Instantaneous Acceleration Given Coordinates in Phase Space of Position and Velocity
extern double calcAccelSetpoint(double currentDistance, double currentVelocity, double errorTolerance, double maxAccel);
#endif  //!__STATE_SPACE_METHODS__H__