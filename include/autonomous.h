#include "main.h"

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

//Tracking constants
#define TRACKING_WHEEL_DIAMETER 2.806 //inches
#define L_TO_MID 0.95 //perpendicular distance from tracking wheel to center
#define R_TO_MID 1.55
#define B_TO_MID 4.1

//Struct definitions

//Position tracking struct and variable
//Units: inches and radians
typedef struct Position {
  double x, y, angle;
  void resetPos() {
    x = y = angle = 0;
  }
  //Constructor
  Position(): x(0), y(0), angle(0) {}
} Position;

//Encoder values struct
//Units: encoder units
typedef struct EncoderVal {
  int left, right, back;
  double angle;
  void resetValues() {
    left = right = back = angle = 0;
  }
  EncoderVal(): left(0), right(0), back(0), angle(0) {}
} EncoderVal;

//Change in values struct
//Units: inches
typedef struct DeltaVal {
  double left, right, back, angle;
  void resetValues() {
    left = right = back = angle = 0;
  }
  DeltaVal(): left(0), right(0), back(0), angle(0) {}
} DeltaVal;

//Struct defining a point on the field
//Units: inches
typedef struct Point {
  double x, y;
  void resetValues() {
    x = y = 0;
  }
  void setValues(double newX, double newY) {
    x = newX;
    y = newY;
  }
  Point(): x(0), y(0) {}
  Point(double initX, double initY): x(initX), y(initY) {}
} Point;

//Extern structs
extern Position robotPos;
extern EncoderVal currentVal, lastVal;
extern DeltaVal deltaVal;

//Other extern variables
extern pros::Mutex driveControl, intakeControl, indexerControl, shooterControl;
extern pros::Mutex pdGetOutput;
extern double numBallsShot, numBallsIntaken;
extern int autonPick;

//Auto function declarations
//auto_functions.cpp
void trackPosition(void*);
void moveShort(double, double, double, bool);
void moveLong(double, double, double, double);
void turnToFace(double, double);
void turnToPoint(double, double, double);

void countBalls(void*);
void intakeShoot(int);
void intakeNoShoot(int);
void intakeNoShoot(void);
void discard(int);
void discard(void);
void pushAway(int);
void pushAway(void);
void stopMotors(void);

//auto_routines.cpp
void movementOne(void*);
void snailOne(void*);
void movementTwo(void*);
void snailTwo(void*);

//auto_utilities.cpp
bool setDriveSafe(double, double);
bool setIntakesSafe(double);
bool setIndexerSafe(double);
bool setShooterSafe(double);

double findDistance(Point, Point);
double findAngle(Point, Point);
double smallestAngle(double, double);
double angleToInches(double);

#endif //_AUTONOMOUS_H_
