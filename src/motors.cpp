#include "main.h"
#include "motors.h"

#define LF_PORT 1
#define RF_PORT 2
#define LB_PORT 3
#define RB_PORT 4

// pros::Motor::Motor (int port, motor_gearset_e_t gearset, bool reverse)
pros::Motor leftFront (LF_PORT, MOTOR_GEARSET_18, false);
pros::Motor rightFront (RF_PORT, MOTOR_GEARSET_18, false);
pros::Motor leftBack (LB_PORT, MOTOR_GEARSET_18, false);
pros::Motor rightBack (RB_PORT, MOTOR_GEARSET_18, false);

// pros::Controller::Controller (id_e_t id)
pros::Controller master (CONTROLLER_MASTER);
