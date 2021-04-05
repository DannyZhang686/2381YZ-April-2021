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

static DriveConfig Z_Bot_Drive = {
    {left_back, {6, 1}},
    {left_front, {9, 0}},
    {right_back, {18, 0}},
    {right_front, {19, 1}},
};
static DriveConfig Y_Bot_Drive = {
    {left_back, {11, 0}},
    {left_front, {20, 0}},
    {right_back, {16, 1}},
    {right_front, {17, 1}},
};
pros::Motor* leftFront = nullptr;
pros::Motor* rightFront = nullptr;
pros::Motor* leftBack = nullptr;
pros::Motor* rightBack = nullptr;


const void InitDrive(DriveConfig config)
{
    leftBack  = new pros::Motor(get<0>(config[left_back]), get<1>(config[left_back]));
    leftFront = new pros::Motor(get<0>(config[left_front]), get<1>(config[left_front]));
    rightBack = new pros::Motor(get<0>(config[right_back]), get<1>(config[right_back]));
    rightFront = new pros::Motor(get<0>(config[right_front]), get<1>(config[right_front]));

    s__t(0, t__s(leftBack->get_actual_velocity()));
}

const void InitMotors(RobotConfig config)
{
    switch(config)
    {
        case Z:
            InitDrive(Z_Bot_Drive);
            break;
        case Y:
            InitDrive(Y_Bot_Drive);
            break;
    }
    InitMotorControllers();
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
