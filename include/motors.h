#include "main.h"
#include "control/motor_controller.hpp"
#include <tuple>
#include <map>

#ifndef _MOTORS_H_
#define _MOTORS_H_

//Extern declarations for motors (and other electronics)
extern pros::Motor* leftFront;
extern pros::Motor* rightFront;
extern pros::Motor* leftBack;
extern pros::Motor* rightBack;

static Motor_Controller* _left_front_motor_controller = nullptr;
static Motor_Controller* _left_back_motor_controller = nullptr;
static Motor_Controller* _right_front_motor_controller = nullptr;
static Motor_Controller* _right_back_motor_controller = nullptr;


enum Motor_Ref
{
    left_back = 0,
    left_front,
    right_back,
    right_front
};

enum RobotConfig
{
    Z = 0,
    Y
};

typedef std::tuple<int, bool> MotorConfig; // Port, Orientation

typedef std::map<Motor_Ref, MotorConfig> DriveConfig;
const void InitMotors(RobotConfig config = Z);
const void InitMotorControllers(void);


extern pros::Motor leftIntake;
extern pros::Motor rightIntake;
extern pros::Motor indexer;
extern pros::Motor shooter;

extern pros::Imu leftIMU;
extern pros::Imu rightIMU;

// extern pros::ADIEncoder rightTracking;
extern pros::ADIEncoder backTracking;
extern pros::ADIEncoder leftTracking;

extern pros::ADIAnalogIn tLineSensor;
extern pros::ADIAnalogIn bLineSensor;

extern pros::Controller master;

#endif //_MOTORS_H_
