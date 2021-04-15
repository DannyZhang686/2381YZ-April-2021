//This file should ONLY contain calls for functions/tasks.
//Everything else of relevance is in autonomous_calls.cpp.

#include "main.h"
#include "autonomous.h"
#include "utilities.h"

void autonomous() {
	#ifndef ALREADY_CALLED
	#define ALREADY_CALLED
		//Initialize tracking and ball-counting tasks (used across all autons)
		pros::Task tracking(trackPosition, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tracking");		//Position tracking task
		pros::Task ballCounting(countBalls, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Count Balls");	//Ball counting task
		pros::Task autoMovement(movementOne, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "AutoMove");	//Robot movement task
		pros::Task autoBallHandling(snailOne, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "AutoSnail");	//Ball control task
	#endif //ALREADY_CALLED
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
