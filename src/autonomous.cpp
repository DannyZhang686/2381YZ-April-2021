//This file should ONLY contain calls for functions/tasks.
//Everything else of relevance is in autonomous_calls.cpp.

#include "main.h"
#include "autonomous.h"
#include "utilities.h"

void autonomous() {
	pros::Task tracking(trackPosition, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tracking");
  if (autonPick == 1) {
		pros::Task auto1a(movementOne, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Auto1Move");
		pros::Task auto1b(snailOne, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Auto1Snail");
  }
	else if (autonPick == 2) {
		pros::Task auto2a(movementTwo, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Auto2Move");
		pros::Task auto2b(snailTwo, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Auto2Snail");
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
