#include "main.h"
#include "motors.h"
#include <tuple>
#include <map>

//Motor ports
#define LF_PORT 9
#define RF_PORT 19
#define LB_PORT 6
#define RB_PORT 18
#define LI_PORT 10
#define RI_PORT 20
#define INDEXER_PORT 2
#define SHOOTER_PORT 1

//IMU ports
#define L_IMU_PORT 4
#define R_IMU_PORT 11

//Encoder and line sensor ports
//In encoder ports, the "out" port is always
//the port immediately following the "in" port
// #define RIGHT_IN 7
// #define RIGHT_OUT 8
#define BACK_IN 5
#define BACK_OUT 6
#define LEFT_IN 7
#define LEFT_OUT 8

#define TOP_LINE 2
#define BOTTOM_LINE 1

//Drive
//Motor reversing accounts for the way the robot is built

using namespace std;
using namespace pros;

pros::Motor *leftFront = nullptr;
pros::Motor *rightFront = nullptr;
pros::Motor *leftBack = nullptr;
pros::Motor *rightBack = nullptr;

const void InitDrive(DriveConfig config)
{
    leftBack = new pros::Motor(get<0>(config[left_back]), get<1>(config[left_back]));
    leftFront = new pros::Motor(get<0>(config[left_front]), get<1>(config[left_front]));
    rightBack = new pros::Motor(get<0>(config[right_back]), get<1>(config[right_back]));
    rightFront = new pros::Motor(get<0>(config[right_front]), get<1>(config[right_front]));

    s__t(0, t__s(leftBack->get_actual_velocity()));
}

const void InitMotors(ConfigOptions config)
{
    switch (config)
    {
    case Y:
        InitDrive(Y_Bot_Drive);
        InitMotorControllers(Y_Bot_Drive_Config);
        break;
    case E:
        InitDrive(Evan_Bot_Drive);
        InitMotorControllers(Z_Bot_Drive_Config);
        break;
    default:
        InitDrive(Z_Bot_Drive);
        InitMotorControllers(Z_Bot_Drive_Config);
        break;
    }
}

//Intakes
pros::Motor leftIntake(LI_PORT, false);
pros::Motor rightIntake(RI_PORT, true);

//Other
pros::Motor indexer(INDEXER_PORT, false);
pros::Motor shooter(SHOOTER_PORT, false);

//IMUs
pros::Imu leftIMU(L_IMU_PORT);
pros::Imu rightIMU(R_IMU_PORT);

//ADI (Encoders and line sensors)
pros::ADIEncoder leftTracking(LEFT_IN, LEFT_OUT, true);
// pros::ADIEncoder rightTracking (RIGHT_IN, RIGHT_OUT, true);
pros::ADIEncoder backTracking(BACK_IN, BACK_OUT, true);

pros::ADIAnalogIn tLineSensor(TOP_LINE);
pros::ADIAnalogIn bLineSensor(BOTTOM_LINE);

// pros::Controller::Controller (id_e_t id)
pros::Controller master(CONTROLLER_MASTER);
