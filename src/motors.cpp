#include "main.h"
#include "motors.h"

#define LF_PORT 9
#define RF_PORT 6
#define LB_PORT 5
#define RB_PORT 3
#define LI_PORT 10
#define RI_PORT 20
#define INDEXER_PORT 2
#define SHOOTER_PORT 1

// pros::Motor::Motor (int port, motor_gearset_e_t gearset, bool reverse)
//Drive
//false and true reverses account for the way the drive is built
pros::Motor leftFront (LF_PORT, MOTOR_GEARSET_18, false);
pros::Motor rightFront (RF_PORT, MOTOR_GEARSET_18, true);
pros::Motor leftBack (LB_PORT, MOTOR_GEARSET_18, true);
pros::Motor rightBack (RB_PORT, MOTOR_GEARSET_18, false);

//Intakes
pros::Motor leftIntake (LI_PORT, MOTOR_GEARSET_18, false);
pros::Motor rightIntake (RI_PORT, MOTOR_GEARSET_18, false);

//Other
pros::Motor indexer (INDEXER_PORT, MOTOR_GEARSET_36, false);
pros::Motor shooter (SHOOTER_PORT, MOTOR_GEARSET_18, false);

// pros::Controller::Controller (id_e_t id)
pros::Controller master (CONTROLLER_MASTER);
