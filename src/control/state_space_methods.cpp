#include "main.h"
#include "globals.hpp"
#include "control/model_predictive_control.hpp"
#include "utilities.h"
#include "control/state_space_methods.hpp"
#include "control/model_predictive_control.hpp"


// Calculates Voltage Needed To Reach Given Acceleration Target given the Current Speed and shape of Velocity Exponential Curve.
double calcVoltageSetpoint(double targetAccel, double decayExponent, double currentSpeed, double maxSpeed)
// Solves for voltage given the differential equation x`` = a * V - b x`,
// where x`` is target accel, x` is current velocity, V is voltage and b = e ^ decayExponent.
// `maxSpeed` is the steady state velocity of the system when the motors are set to maximum voltage.

// Returns a value from -1 to 1, which is proportional to the voltage input of the system.
// Specifically, the calculated motor voltage = MAX_VOLTAGE * voltageCoefficient.
	{
	double yIntercept = targetAccel / decayExponent;
	return minMaxMod((currentSpeed - yIntercept) / maxSpeed, 1);
	}

// Calculates Instantaneous Acceleration Given Coordinates in Phase Space of Position and Velocity
double calcAccelSetpoint(double currentDistance, double currentVelocity, double errorTolerance, double maxAccel)
	{
	double accelCoeff;
	if (currentVelocity > 0 && currentDistance > 0) // Quadrant 1 Exclusive
		{
		accelCoeff = -1;
		}
	else if (currentVelocity >= 0 && currentDistance <= 0) //Quadrant 2 Inclusive
		{
		double expectedDeaccelVelocity = sqrt(-maxAccel * (currentDistance));
		// Maximum deaccel curve that goes into (0,0), if the current phase is between max deaccel curve and y-axis.
		if (currentVelocity >= expectedDeaccelVelocity)
			{
			accelCoeff = -1;
			}
		else
			{
			// Under max deaccel curve
			if (abs(currentDistance) < errorTolerance)
				{
				// inside error tolerance zone, between max deacell curve and x-axis.
				accelCoeff = 0;
				}
			else {
				double bufferDeaccelVelocity = expectedDeaccelVelocity - sqrt(abs(maxAccel * errorTolerance));
				if (currentVelocity >= bufferDeaccelVelocity)
					{
					// Inside buffer zone
					accelCoeff = 0;
					}
				else if (currentVelocity < bufferDeaccelVelocity)
					{
					// Under buffer zone, continue accelerating
					accelCoeff = 1;
					}
				}
			}
		}
	else if (currentVelocity < 0 && currentDistance < 0) // Quadrant 3 Exclusive
		{
		accelCoeff = 1;
		}
	else if (currentVelocity <= 0 && currentDistance >= 0) // Quadrant 4 Inclusive
		{
		double expectedDeaccelVelocity = -sqrt(maxAccel * (currentDistance));
		// Maximum deaccel curve that goes into (0,0), if the current phase is between max deaccel curve and y-axis.
		if (currentVelocity <= expectedDeaccelVelocity)
			{
			accelCoeff = 1;
			}
		else
			{
			// Under max deaccel curve
			if (abs(currentDistance) < errorTolerance)
				{
				// inside error tolerance zone, between max deacell curve and x-axis.
				accelCoeff = 0;
				}
			else {
				double bufferDeaccelVelocity = expectedDeaccelVelocity + sqrt(abs(maxAccel * errorTolerance));

				if (currentVelocity <= bufferDeaccelVelocity)
					{
					// Inside buffer zone
					accelCoeff = 0;
					}
				else if (currentVelocity > bufferDeaccelVelocity)
					{
					// Under buffer zone, continue accelerating
					accelCoeff = -1;
					}
				}
			}
		}
	else
		{
		// Uhh something fucked up.
		accelCoeff = 0;
		}

	return accelCoeff * maxAccel;
	}
