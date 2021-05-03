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
AutonControl& auton_control = AutonControl::instance();
MasterController* master_control = MasterController::instance();
bool STOP = false;
Position_Tracker* position_tracker = Position_Tracker::instance();
DriveMode activeDriveMode = PidMode;

void initialize()
{
	pros::lcd::initialize();
	InitMotors(L);

	position_tracker->Create();

	pros::Task tracking(trackPosition, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Tracking");
	// pros::Task shooter(shooterSpin, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Shooter");
	// pros::Task intake(intakeSpin, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake");
	// pros::Task indexer(indexerSpin, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Indexer");

	// pros::Task ballCounting(countBalls, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Count Balls");
	pros::Task PidDrive(PID_Drive, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PID Drive");




	using namespace Auton;
	// AutoSequence* CUS_FULL = AutoSequence::FromTasks({*CUS_Q1,
	// 												  *CUS_Q2,
	// 												  *CUS_Q3,
	// 												  *CUS_Q4});

	// auton_control.define_auton(AutonControl::CUS_ALL, CUS_FULL);
	auton_control.define_auton(AutonControl::CUS_Q1, CUS_Q1);
	// auton_control.define_auton(AutonControl::CUS_Q2, CUS_Q2);
	// auton_control.define_auton(AutonControl::CUS_Q3, CUS_Q3);
	// auton_control.define_auton(AutonControl::CUS_Q4, CUS_Q4);

	auton_control.define_auton(AutonControl::TestAuton, AT_Test_Ultras);
	auton_control.select_auton(AutonControl::CUS_Q1);

	//initialize GUI task (for refreshes)
	//need to test this to see if it carries over
	// pros::Task brainScreen(gui, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "GUI");
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
