#include "main.h"
#include "autonomous.h"
#include "gui.h"
#include "motors.h"
#include "globals.hpp"
#include "autonomous/auton_control.hpp"
#include "autonomous/global_sequences.hpp"
#include "opcontrol.h"
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

bool Competition_Env = false;
AutonControl *auton_control = AutonControl::instance();
MasterController *master_control = MasterController::instance();
bool STOP = false;
Position_Tracker *position_tracker = Position_Tracker::instance();
DriveMode activeDriveMode = PidMode;

void initialize()
{
	pros::lcd::initialize();
	InitMotors(E);

	position_tracker->Create();

	pros::Task tracking(trackPosition, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tracking");
	pros::Task shooter(shooterSpin, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Shooter");
	pros::Task intake(intakeSpin, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake");
	pros::Task indexer(indexerSpin, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Indexer");

	// // pros::Task ballCounting(countBalls, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Count Balls");
	pros::Task PidDrive(PID_Drive, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Drive");



	using namespace Auton;
	auton_control->define_auton(AutonControl::TestAuton, AT_Test_Ultras);
	auton_control->select_auton(AutonControl::TestAuton);

	//initialize GUI task (for refreshes)
	//need to test this to see if it carries over
	// pros::Task brainScreen(gui, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "GUI");

	// tLineSensor.calibrate(); //Blocking, takes around 500 ms
	// bLineSensor.calibrate();
	// pros::delay(3000); //Let everything init properly
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

void competition_initialize()
{
	bool Competition_Env = true;
}
