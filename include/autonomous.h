#include "main.h"
#include "autonomous/pathing.hpp"
#include "autonomous/auto_task.hpp"
#include "autonomous/position_tracker.hpp"
#include "autonomous/auto_timer.hpp"
#include "autonomous/auton_control.hpp"
#include "autonomous/auto_timer.hpp"

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

#define TRACKING_WHEEL_DIAMETER 2.806 //inches

//Position tracking struct and variable
//Units: inches and radians

extern double numBallsShot, numBallsIntaken;

//Auto function declarations
//auto_functions.cpp
void trackPosition(void*);

void countBalls(void*);
void intakeShoot(int, int);
void intakeNoShoot(int, double);
void intakeNoShoot(double);
void discardFront(int);
void discardFront(void);
void discardBack(int);
void discardBack(void);
void pushAway(int);
void pushAway(void);
void stopMotors(void);

//auto_utilities.cpp
bool setDriveSafe(double, double);
bool setIntakesSafe(double);
bool setIndexerSafe(double);
bool setShooterSafe(double);

double smallestAngle(double, double);
double angleToInches(double);

#define AUTO_INTAKE_VEL 200
#define AUTO_INDEXER_VEL 200
#define AUTO_SHOOTER_VEL 200

#endif //_AUTONOMOUS_H_
