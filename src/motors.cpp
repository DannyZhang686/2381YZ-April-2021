#include "main.h"
#include "motors.h"

//Motor ports
#define LF_PORT 9
#define RF_PORT 6
#define LB_PORT 5
#define RB_PORT 3
#define LI_PORT 10
#define RI_PORT 20
#define INDEXER_PORT 2
#define SHOOTER_PORT 1

//IMU ports
#define L_IMU_PORT 0
#define R_IMU_PORT 0

//Encoder and line sensor ports
//In encoder ports, the "out" port is always
//the port immediately following the "in" port
#define RIGHT_IN 0
#define RIGHT_OUT RIGHT_IN+1
#define BACK_IN 0
#define BACK_OUT BACK_IN+1
#define LEFT_IN 0
#define LEFT_OUT LEFT_IN+1

#define LINE_PORT 0

//Drive
//Motor reversing accounts for the way the robot is built
pros::Motor leftFront (LF_PORT, false);
pros::Motor rightFront (RF_PORT, true);
pros::Motor leftBack (LB_PORT, true);
pros::Motor rightBack (RB_PORT, false);

//Intakes
pros::Motor leftIntake (LI_PORT, false);
pros::Motor rightIntake (RI_PORT, true);

//Other
pros::Motor indexer (INDEXER_PORT, false);
pros::Motor shooter (SHOOTER_PORT, false);

//IMUs
pros::Imu leftIMU (L_IMU_PORT);
pros::Imu rightIMU (R_IMU_PORT);

//ADI (Encoders and line sensors)
pros::ADIEncoder leftTracking (LEFT_IN, LEFT_OUT, false);
pros::ADIEncoder rightTracking (RIGHT_IN, RIGHT_OUT, true);
pros::ADIEncoder backTracking (BACK_IN, BACK_OUT, false);

pros::ADIAnalogIn lineSensor (LINE_PORT);

// pros::Controller::Controller (id_e_t id)
pros::Controller master (CONTROLLER_MASTER);
