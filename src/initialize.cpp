#include "main.h"
#include "autonomous.h"
#include "gui.h"
#include "motors.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	pros::lcd::initialize();
	InitMotors(Z);
	//initialize GUI task (for refreshes)
	//need to test this to see if it carries over
	// pros::Task brainScreen(gui, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "GUI");

	leftIMU.reset(); //Non-blocking but takes around 2000 ms
	rightIMU.reset();
	tLineSensor.calibrate(); //Blocking, takes around 500 ms
	bLineSensor.calibrate();
	pros::delay(3000); //Let everything init properly
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

void competition_initialize() {}
