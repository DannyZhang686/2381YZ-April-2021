#include "globals.hpp"
#include "pathing.hpp"
#include "autonomous.h"
#include "pid.h"
#include "opcontrol.h"
#include "utilities.h"
#include "motors.h"
#include "autonomous/functional_tasks.hpp"

using namespace std;

AutoTask SingleRun(std::function<void(void)> run)
{
	return AutoTask(run, [](void) -> bool { return true; });
}

namespace TurnToPoint
{
	double targetAngle;  //The desired angle, which is constantly updated
	double angleDiff;    //The angle (-π to π) to travel to face the final angle
	double tAngleInches; //travellingAngle converted to a value in inches (for PD purposes)
	int time;
	bool setTime;

	static double turnCoeff = 0.95;
}

double calcVoltageCoefficient(double maxSpeed, double currentSpeed, double decayExponent, double targetAcceleration)
// Solves for voltage given the differential equation x`` = a * V - b x`,
// where x`` is target accel, x` is current velocity, V is voltage and b = e ^ decayExponent.
// `maxSpeed` is the steady state velocity of the system when the motors are set to maximum voltage.

// Returns a value from -1 to 1, which is proportional to the voltage input of the system.
// Specifically, the calculated motor voltage = MAX_VOLTAGE * voltageCoefficient.
{
	double yIntercept = targetAcceleration / decayExponent;
	return (currentSpeed - yIntercept) / maxSpeed;
}

const double MAX_SPEED = 12 * M_PI / 180; // 0.191986217719
const double MAX_ACCEL = 1 * M_PI / 180; // 0.00872664625997
const double EXPONENT = -0.1135;

double calcSquareRoot(double currentDistance, double currentVelocity, double accel = 1.0)
{
	double expectedVelocity = -sqrt(abs(MAX_ACCEL * accel * currentDistance)) * getSignOf(currentDistance);
	return currentVelocity / expectedVelocity;
}

double calcAccelTarget(double currentDistance, double currentVelocity, double errorTolerance, double maxAccel)
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

AutoTask TurnToPointSMOOTH(Point targetPoint, double power, double accel)
{

	double& angleDiff = *(new double()), currentAngVel = *(new double()), initialAngle = *(new double());
	double& initialDistance = *(new double());

	int& iteration = *(new int(0));

	auto init = [&, power, accel, targetPoint]
	{
		double targetAngle = arg(targetPoint - position_tracker->Get_Position());
		initialAngle = position_tracker->Get_Angle();
		initialDistance = NormalizeAngle(targetAngle - initialAngle);
	};

	auto run = [&, targetPoint, power, accel](void) -> void
	{
		double targetAngle = arg(targetPoint - position_tracker->Get_Position());
		double currentAngle = position_tracker->Get_Angle();
		angleDiff = NormalizeAngle(currentAngle - targetAngle);

		currentAngVel = position_tracker->Get_Ang_Vel();
		double travelledDistance = NormalizeAngle(currentAngle - initialAngle);

		double accelTarget = calcAccelTarget(angleDiff, currentAngVel, 0.1, accel * MAX_ACCEL);

		double motorPower = 127 * calcVoltageCoefficient(MAX_SPEED, currentAngVel, EXPONENT, accelTarget);
		Set_Drive_Direct(-motorPower, -motorPower, motorPower, motorPower);
		
#define DEBUG
#ifdef DEBUG
		std::string a = "" + t__s(iteration) + "x" + t__s(currentAngVel) + "x" + t__s(accelTarget) + "x" + t__s(angleDiff) + "\n";
		printf(a.c_str());
#undef DEBUG
#endif
		(iteration)++;
	};

	auto done = [&]() -> bool {
		return (abs(angleDiff) < abs(initialDistance) / 2 && abs(currentAngVel) < 0.05 * MAX_SPEED);
	};

	auto kill = [&]
	{
		Set_Drive_Direct(0, 0, 0, 0);
		delete (&angleDiff, &currentAngVel, &initialAngle, &iteration);
	};

	return AutoTask::SyncTask(run, done, init, kill);
}


AutoTask TurnToPointTask(Point target, double maxError, double turnSpeed)
{
	using namespace TurnToPoint;

	auto init = [&](void) -> void {
		time = 0;
		setTime = false;
	};

	auto runFn = [&, target, maxError, turnSpeed]() -> void {
		targetAngle = arg(target - position_tracker->Get_Position());

		angleDiff = NormalizeAngle(targetAngle - position_tracker->Get_Angle());

		auto input = (abs(angleDiff) > M_PI / 2) ? turnSpeed * getSignOf(angleDiff) : turnSpeed * (turnCoeff * sin(angleDiff) / pow(pow(sin(angleDiff), 2.0), -0.1) + (1 - turnCoeff) * abs(sin(angleDiff)) / sin(angleDiff));
		Set_Drive(-input, -input, input, input);
		s__t(0, "TURN:" + t__s(targetAngle) + " " + t__s(position_tracker->Get_Angle()) + " " + t__s(angleDiff));
		s__t(1, "TURN_VEL:" + t__s(input) + " " + t__s(position_tracker->Get_Ang_Vel()));
	};

	auto doneFn = [&, maxError]() -> bool {
		return (abs(angleDiff) < maxError && abs(position_tracker->Get_Ang_Vel()) < maxError);
	};
	auto kill = [] {
		Set_Drive(0, 0, 0, 0);
	};
	return AutoTask::SyncTask(runFn, doneFn, init, kill);
}

AutoTask TurnToAngleTask(double heading, double maxError)
{
	using namespace TurnToPoint;

	auto init = [&](void) -> void {
		time = 0;
		setTime = false;
	};

	auto runFn = [&, heading, maxError]() -> void {
		// targetAngle = heading;

		// travellingAngle = -smallestAngle(position_tracker->Get_Angle(), targetAngle);
		// tAngleInches = angleToInches(travellingAngle);

		// double leftOutput = 0, rightOutput = 0; //Power output (0-200) for each side of the robot
		// // leftOutput = turnCalc(leftTurn.getOutput(0, 20 * tAngleInches)); //Setpoint distance value for PD
		// rightOutput = -rightTurn.getOutput(0, 20 * tAngleInches);
		// // rightOutput = -leftOutput;
		// setDriveSafe(-rightOutput, rightOutput);
		// // Set_Drive(leftOutput, leftOutput, -leftOutput, -leftOutput);
		// s__t(4, t__s(targetAngle) + " " + t__s(position_tracker->Get_Angle()) + " " + t__s(travellingAngle));
	};

	auto doneFn = [&, maxError]() -> bool {
		// if ((!setTime) && (fabs(travellingAngle) < maxError))
		// {
		//     time = pros::millis();
		//     setTime = true;
		// }
		// else if ((time != 0) && (pros::millis() - time > 250))
		// {
		//     return true;
		// }
		return false;
	};
	auto kill = [] {
		Set_Drive(0, 0, 0, 0);
	};
	return AutoTask::SyncTask(runFn, doneFn, init, kill);
}

namespace IntakeShoot
{
	double initNumBallsShot;    //Number of balls shot before this point
	double initNumBallsIntaken; //Number of balls intaken before this point
	bool doneIntaking, doneShooting;
	int time;
	bool setTime;
}

AutoTask IntakeShootTask(int numBallsIn, int numBallsOut)
{
	using namespace IntakeShoot;
	auto init = [&, numBallsIn, numBallsOut](void) -> void {
		initNumBallsShot = numBallsShot;       //Number of balls shot before this point
		initNumBallsIntaken = numBallsIntaken; //Number of balls intaken before this point
		doneIntaking = false, doneShooting = false;
		time = 0;
		setTime = false;
		setIntakesSafe(AUTO_INTAKE_VEL);
		setIndexerSafe(AUTO_INDEXER_VEL);
		setShooterSafe(AUTO_SHOOTER_VEL);
	};

	auto run = [&, numBallsIn, numBallsOut](void) -> void {
		if ((!setTime) && (initNumBallsShot + numBallsOut <= numBallsShot))
		{
			//Spin the shooter the other way instead, after a short delay
			time = pros::millis();
			setTime = true;
			// s__t(3, "time set");
		}
		else if ((time != 0) && (pros::millis() - time > 0))
		{
			setShooterSafe(-AUTO_SHOOTER_VEL);
			doneShooting = true;
			// s__t(4, "");
		}
		if ((numBallsIn == 0) || (initNumBallsIntaken + numBallsIn + 0.5 <= numBallsIntaken))
		{
			//Stop the intakes
			setIntakesSafe(0);
			doneIntaking = true;
		}
	};

	auto done = [&, numBallsIn, numBallsOut](void) -> bool {
		if (doneShooting && doneIntaking)
		{
			return true;
		}
		return false;
	};

	auto kill = [](void) -> void {
		setIntakesSafe(0);
	};

	return AutoTask::SyncTask(run, done, init, kill);
}
