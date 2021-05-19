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
		std::string a = "" + t__s(iteration) + "x" + t__s(currentAngVel) + "x" + t__s(accelTarget) + "x" + t__s(angleDiff) + "x" + t__s(motorPower) + "\n";
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

AutoTask IntakeShootTask(int ballsDescored)
{
	//ONLY SUPPORTS VALUES FROM 0 TO 2
	double& initialBottom = *(new double()), &initialMiddle = *(new double()), &initialTop = *(new double());
	bool &isDoneTop = *(new bool()), &isDoneBottom = *(new bool());
	int &ballsScored = *(new int());

	auto init = [&]
	{
		initialBottom = numBallsBottom;
		initialMiddle = numBallsMiddle;
		initialTop = numBallsTop;
		isDoneTop = isDoneBottom = false;
		ballsScored = numBallsInRobot;
	};

	auto run = [&, ballsDescored](void) -> void
	{
		double curBottom = numBallsBottom - initialBottom;
		double curMiddle = numBallsMiddle - initialMiddle;
		double curTop = numBallsTop - initialTop;

		if (ballsScored == 0) {
		  isDoneTop = true;
		}
		else if (ballsScored == 1) {
		  if (curTop >= 1) {
		    setShooterSafe(0);
		    isDoneTop = true;
		  }
		  else {
		    setShooterSafe(-AUTO_SHOOTER_VEL);
		  }
		}
		else if (ballsScored == 2) {
		  if (curTop >= 2) {
		    setShooterSafe(0);
		    setIndexerSafe(0);
		    isDoneTop = true;
		  }
		  else if (curTop >= 0.5) { //TODO: can honestly probably change this to mLineSensor updating bc there's such a huge delay rn
		    setShooterSafe(-AUTO_SHOOTER_VEL);
		    if (curMiddle >= 1) {
		      setIndexerSafe(0);
		    }
		    else {
		      setIndexerSafe(AUTO_INDEXER_VEL);
		    }
		  }
		  else {
		    setShooterSafe(-AUTO_SHOOTER_VEL);
		  }
		}

		if (ballsDescored == 0) {
		  isDoneBottom = true;
		}
		else if (ballsDescored == 1) {
		  if (curBottom >= 0.5) {
		    setIntakesSafe(0);
		    isDoneBottom = true;
		  }
		  else {
		    setIntakesSafe(AUTO_INTAKE_VEL);
		  }
		}
		else if (ballsDescored == 2) {
		  if (curBottom >= 1.5) {
		    setIntakesSafe(0);
		    if (ballsScored != 2) {
					setIndexerSafe(0);
				}
		    isDoneBottom = true;
		  }
		  else {
		    setIntakesSafe(AUTO_INTAKE_VEL);
		    if (ballsScored != 2) {
					setIndexerSafe(1000);
				}
		  }
		}
	};

	auto done = [&]() -> bool {
		return (isDoneTop && isDoneBottom);
	};

	auto kill = [&]
	{
		stopMotors();
		delete (&initialBottom, &initialMiddle, &initialTop, &isDoneTop, &isDoneBottom, &ballsScored);
	};

	return AutoTask::SyncTask(run, done, init, kill);
}
