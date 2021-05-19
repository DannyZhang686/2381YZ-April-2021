#include "globals.hpp"
#include "pathing.hpp"
#include "autonomous.h"
#include "pid.h"
#include "opcontrol.h"
#include "utilities.h"
#include "motors.h"

#include "autonomous/functional_tasks.hpp"
#include "control/model_predictive_control.hpp"
#include "config/profiling.config.hpp"


using namespace std;

AutoTask SingleRun(std::function<void(void)> run)
	{
	return AutoTask(run, [](void) -> bool { return true; });
	}

AutoTask TurnToPointSmooth(Point targetPoint, double accel, double errorTolerance)
	{

	double& angleDiff = *(new double()), & currentAngVel = *(new double()), & initialAngle = *(new double());
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

		double motorPower = 127 * calcVoltageSetpoint(accelTarget, TURN_EXPONENT_CONST, currentAngVel, MAX_TURN_SPEED_CONST);
		Set_Drive_Direct(-motorPower, -motorPower, motorPower, motorPower);

		// #define DEBUG
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
