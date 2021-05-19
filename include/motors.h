     

#ifndef _MOTORS_H_
#define _MOTORS_H_

#include "main.h"
#include "control/motor_controller.hpp"
#include "config/config_types.hpp"
#include "inertial_module.hpp"

#include <tuple>
#include <map>


//Extern declarations for motors (and other electronics)
extern pros::Motor* leftFront;
extern pros::Motor* rightFront;
extern pros::Motor* leftBack;
extern pros::Motor* rightBack;

static Motor_Controller* _left_front_motor_controller = nullptr;
static Motor_Controller* _left_back_motor_controller = nullptr;
static Motor_Controller* _right_front_motor_controller = nullptr;
static Motor_Controller* _right_back_motor_controller = nullptr;


const void InitMotors(ConfigOptions config = Z);
const void InitMotorControllers(DrivePidConfig DrivePidConfig );


extern pros::Motor leftIntake;
extern pros::Motor rightIntake;
extern pros::Motor indexer;
extern pros::Motor shooter;


// extern pros::ADIEncoder rightTracking;
extern pros::ADIEncoder* backTracking;
extern pros::ADIEncoder* leftTracking;

extern pros::ADIAnalogIn tLineSensor;
extern pros::ADIAnalogIn mLineSensor;
extern pros::ADIAnalogIn mbLineSensor;
extern pros::ADIAnalogIn bLineSensor;

extern pros::Controller master;

#endif //_MOTORS_H_
