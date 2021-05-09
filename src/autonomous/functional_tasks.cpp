#include "globals.hpp"
#include "pathing.hpp"
#include "autonomous.h"
#include "pid.h"
#include "opcontrol.h"
#include "utilities.h"
#include "motors.h"
#include "autonomous/functional_tasks.hpp"

#define ROBOT_L
// #define ROBOT_Z


#ifdef ROBOT_L
const double MAX_TURN_SPEED_CONST = 12 * M_PI / 180; // 0.191986217719
const double TURN_EXPONENT = -0.1135;
// This is the slope of the graph if you set motors to 127 at time = 0.
// v(t) = -a*e^{bx} + a
// d/dt v(t) = -ab * e^{bx}
// d/dt v(0) = -ab
const double MAX_TURN_ACCEL_CONST = - TURN_EXPONENT * MAX_TURN_SPEED_CONST; // 0.02377
#undef ROBOT_L
#endif


#ifdef ROBOT_Z
const double MAX_TURN_SPEED_CONST = 12 * M_PI / 180; // 0.191986217719
const double TURN_EXPONENT = -0.1135;
// This is the slope of the graph if you set motors to 127 at time = 0.
// v(t) = -a*e^{bx} + a
// d/dt v(t) = -ab * e^{bx}
// d/dt v(0) = -ab
const double MAX_TURN_ACCEL_CONST = - TURN_EXPONENT * MAX_TURN_SPEED_CONST; // 0.02377
#undef ROBOT_Z
#endif

using namespace std;

AutoTask SingleRun(std::function<void(void)> run)
{
	return AutoTask(run, [](void) -> bool { return true; });
}

// Calculates Voltage Needed To Reach Given Acceleration Target given the Current Speed and shape of Velocity Exponential Curve.
double calcVoltageSetpoint(double targetAccel,  double decayExponent, double currentSpeed, double maxSpeed)
// Solves for voltage given the differential equation x`` = a * V - b x`,
// where x`` is target accel, x` is current velocity, V is voltage and b = e ^ decayExponent.
// `maxSpeed` is the steady state velocity of the system when the motors are set to maximum voltage.

// Returns a value from -1 to 1, which is proportional to the voltage input of the system.
// Specifically, the calculated motor voltage = MAX_VOLTAGE * voltageCoefficient.
{
	double yIntercept = targetAccel / decayExponent;
	return (currentSpeed - yIntercept) / maxSpeed;
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

AutoTask TurnToPointSmooth(Point targetPoint, double accel, double errorTolerance)
{

	double& angleDiff = *(new double()), &currentAngVel = *(new double()), &initialAngle = *(new double());
	double& initialDistance = *(new double());

	int& iteration = *(new int(0));

	auto init = [&, accel, targetPoint]
	{
		double targetAngle = arg(targetPoint - position_tracker->Get_Position());
		initialAngle = position_tracker->Get_Angle();
		initialDistance = NormalizeAngle(targetAngle - initialAngle);
	};

	auto run = [&, targetPoint, accel, errorTolerance](void) -> void
	{
		double targetAngle = arg(targetPoint - position_tracker->Get_Position());
		double currentAngle = position_tracker->Get_Angle();
		angleDiff = NormalizeAngle(currentAngle - targetAngle);

		currentAngVel = position_tracker->Get_Ang_Vel();

		double accelTarget = calcAccelSetpoint(angleDiff, currentAngVel, errorTolerance, accel * MAX_TURN_ACCEL_CONST);

		double motorPower = 127 * calcVoltageSetpoint(accelTarget, TURN_EXPONENT, currentAngVel, MAX_TURN_SPEED_CONST);
		Set_Drive_Direct(-motorPower, -motorPower, motorPower, motorPower);
		
#define DEBUG
#ifdef DEBUG
		std::string a = "" + t__s(iteration) + "x" + t__s(currentAngVel) + "x" + t__s(accelTarget) + "x" + t__s(angleDiff) + "\n";
		printf(a.c_str());
#undef DEBUG
#endif
		(iteration)++;
	};

	auto done = [&, errorTolerance]() -> bool {
		return (abs(angleDiff) < abs(initialDistance) / 2 && abs(currentAngVel) < 0.05 * MAX_TURN_SPEED_CONST);
	};

	auto kill = [&]
	{
		Set_Drive_Direct(0, 0, 0, 0);
		delete (&angleDiff, &currentAngVel, &initialAngle, &iteration);
	};

	return AutoTask::SyncTask(run, done, init, kill);
}

