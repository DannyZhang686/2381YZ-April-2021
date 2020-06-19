#include "main.h"
#include "motors.h"

#define LF_PORT 0
#define RF_PORT 0
#define LB_PORT 5
#define RB_PORT 3
#define LI_PORT 10
#define RI_PORT 20
#define IN_PORT 2
#define FW_PORT 1

// pros::Motor::Motor (int port, motor_gearset_e_t gearset, bool reverse)
//Drive
pros::Motor leftFront (LF_PORT, MOTOR_GEARSET_18, false);
pros::Motor rightFront (RF_PORT, MOTOR_GEARSET_18, false);
pros::Motor leftBack (LB_PORT, MOTOR_GEARSET_18, false);
pros::Motor rightBack (RB_PORT, MOTOR_GEARSET_18, false);

//Intakes
pros::Motor leftIntake (LI_PORT, MOTOR_GEARSET_18, false);
pros::Motor rightIntake (RI_PORT, MOTOR_GEARSET_18, false);

//Other
pros::Motor indexer (IN_PORT, MOTOR_GEARSET_18, false);
pros::Motor flywheel (FW_PORT, MOTOR_GEARSET_36, false);

// pros::Controller::Controller (id_e_t id)
pros::Controller master (CONTROLLER_MASTER);
