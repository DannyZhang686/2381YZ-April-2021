#include "main.h"

#ifndef _MOTORS_H_
#define _MOTORS_H_


extern pros::Motor leftFront;
extern pros::Motor rightFront;
extern pros::Motor leftBack;
extern pros::Motor rightBack;
extern pros::Motor leftIntake;
extern pros::Motor rightIntake;
extern pros::Motor indexer;
extern pros::Motor shooter;

extern pros::Imu leftIMU;
extern pros::Imu rightIMU;

extern pros::ADIEncoder rightTracking;
extern pros::ADIEncoder backTracking;
extern pros::ADIEncoder leftTracking;

extern pros::ADIAnalogIn lineSensor;

extern pros::Controller master;

#endif //_MOTORS_H_
