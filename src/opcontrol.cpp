#include "main.h"
#include "opcontrol.h"
#include "motors.h"

void opcontrol() {
	//this is how to pass variables; it probably won't actually be used though, except in auton
	std::pair<int, int>* driveValues = new std::pair(std::make_pair((int) master.get_analog(ANALOG_LEFT_Y), (int) master.get_analog(ANALOG_RIGHT_X)));
	pros::Task splitArcadeDrive(splitArcade, (void*)(&driveValues), TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drive");
	delete driveValues;
	pros::Task flywheelSpin(flywheel, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel");
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