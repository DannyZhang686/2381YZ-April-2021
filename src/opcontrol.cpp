#include "main.h"
#include "opcontrol.h"
#include "autonomous.h"
#include "motors.h"
#include "pid.h"
#include "globals.hpp"

void driver(void)
{
	Controller_Set_Drive(0, master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X), master.get_analog(ANALOG_RIGHT_Y));
	// splitArcade();
}
void opcontrol()
{
	while (true)
	{
		master_control->run();
		controllerIndexerSpin();

		pros::delay(10);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
