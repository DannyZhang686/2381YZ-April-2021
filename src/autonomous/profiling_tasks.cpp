#include "globals.hpp"
#include "autonomous/pathing.hpp"
#include "autonomous.h"
#include "pid.h"
#include "opcontrol.h"
#include "utilities.h"
#include "motors.h"
#include "autonomous/functional_tasks.hpp"
#include "drive.hpp"
using namespace std;


AutoTask DriveProfileTask(double speed, double time)
	{
	int* iteration;
	auto init = [&](void)->void
		{
		iteration = new int(0);
		};

	auto run = [&, speed](void) -> void
        {
        // auto voltages = Calc_Drive_MPC(speed, speed, true);
        // Set_Drive_Direct(voltages[0], voltages[0], voltages[1], voltages[1]);
        // printf("hello");
        double currentSpeed = abs(position_tracker->Get_Velocity());
        double currentAngVel = position_tracker->Get_Ang_Vel();
        double leftTrackingVel = position_tracker->position_change[Position_Tracker::Tracking_Encoder::right_];
        double rightWheelSpeed = (rightFront->get_actual_velocity() + rightBack->get_actual_velocity())/2;
        double leftWheelSpeed = (leftFront->get_actual_velocity() + leftBack->get_actual_velocity())/2;

        Set_Drive_Direct(127,127, 127, 127);
        double currentWheelSpeed = (abs(leftBack->get_actual_velocity()) + abs(rightBack->get_actual_velocity()) + abs(leftFront->get_actual_velocity()) + abs(rightFront->get_actual_velocity())) / 4;
        s__t(3, t__s(*iteration));
        // std::string a = "" + t__s(*iteration) + "x" + t__s(leftBack->get_actual_velocity()) + "x" + t__s(leftFront->get_actual_velocity()) +  "x" + t__s(rightBack->get_actual_velocity()) + "x" + t__s(rightFront->get_actual_velocity())  + "\n";
        std::string a = "" + t__s(*iteration) + "x" + t__s(currentAngVel) + "x" + t__s(leftTrackingVel) + "x" + t__s(currentSpeed) + "\n";
        printf(a.c_str());
        (*iteration)++;
        };

	auto done = [&](void) -> bool
		{
		return false;
		};

	auto kill = [&](void)->void
		{
		printf("\n------------------\n\n");
		delete iteration;
		};

	return AutoTask::SyncTask(run, done, init, kill).TimeLimit(time);
	}



AutoTask RestProfileTask()
	{
	int* iteration;
	double* wheelSpeed;

	auto init = [&](void)->void
		{
		iteration = new int(0);
		wheelSpeed = new double(0);
		};

	auto run = [&](void) -> void
		{
		Set_Drive_Direct(0, 0, 0, 0);
		double currentSpeed = abs(position_tracker->Get_Velocity());
		double currentAngVel = position_tracker->Get_Ang_Vel() * 180 / M_PI;

		(*wheelSpeed) = (abs(leftBack->get_actual_velocity()) + abs(rightBack->get_actual_velocity()) + abs(leftFront->get_actual_velocity()) + abs(rightFront->get_actual_velocity())) / 4;
		s__t(3, t__s(*iteration));
		std::string a = "" + t__s(*iteration) + "x" + t__s(currentAngVel) + "x" + t__s(*wheelSpeed) + "x" + t__s(currentSpeed) + "\n";

		// printf(a.c_str());
		(*iteration)++;
		};

	auto done = [&](void) -> bool
		{
		return (*wheelSpeed) < 1;
		};

	auto kill = [&](void)->void
		{
		printf("\n------------------\n\n");
		delete iteration, wheelSpeed;
		};

	return AutoTask::SyncTask(run, done, init, kill);
	}
