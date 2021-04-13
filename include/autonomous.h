#include "main.h"

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

//Tracking constants
#define TRACKING_WHEEL_DIAMETER 2.806 //inches
// #define L_TO_MID 1.0625
// #define R_TO_MID 1.4375
// #define B_TO_MID 3.625
// #define L_TO_MID 1.25 //perpendicular distance from tracking wheel to center
// #define R_TO_MID 1.25
// #define B_TO_MID 4
#define L_TO_MID 2.75
#define B_TO_MID 5.75

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
typedef struct OPoint {
  double x, y;
  void resetValues() {
    x = y = 0;
  }
  void setValues(double newX, double newY) {
    x = newX;
    y = newY;
  }
  OPoint(): x(0), y(0) {}
  OPoint(double initX, double initY): x(initX), y(initY) {}
} OPoint;

typedef struct Line {
  OPoint points[1000];
  OPoint findGoalPoint(OPoint curPos, double lookDist) {
    double minDist = 1e9, curDist;
    int closestIndex, goalIndex;

    //Find the closest point to the robot
    for (int i = 0; i < 1000; i++) {
      curDist = (points[i].x - curPos.x) * (points[i].x - curPos.x) + (points[i].y - curPos.y) * (points[i].y - curPos.y);
      if (minDist > curDist) {
        minDist = curDist;
        closestIndex = i;
      }
    }

    //Find the goal point
    minDist = 1e9;
    lookDist = lookDist * lookDist;
    for (int i = closestIndex; i < 1000; i++) {
      curDist = (points[i].x - curPos.x) * (points[i].x - curPos.x) + (points[i].y - curPos.y) * (points[i].y - curPos.y);
      if (minDist > fabs(lookDist - curDist)) {
        minDist = fabs(lookDist - curDist);
        goalIndex = i;
      }
    }
    s__t(7, t__s(closestIndex) + " " + t__s(goalIndex) + " " + t__s(lookDist) + " " + t__s(minDist));
    return points[goalIndex];
  }
  Line() {
  }

  Line(OPoint start, OPoint end) {
    for (int i = 0; i < 1000; i++) {
      points[i].x = start.x + (i+1) * 0.001 * (end.x - start.x);
      points[i].y = start.y + (i+1) * 0.001 * (end.y - start.y);
    }
  }
} Line;

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
void movePurePursuit(double, double, double, double, bool);
void approachGoal(double, double);
void backAway(double, double);
void turnToFace(double, double);
void turnToPoint(double, double, double);
void turnAwayFromPoint(double, double, double);

void countBalls(void*);
void intakeShoot(int, int);
void intakeNoShoot(int, double);
void intakeNoShoot(double);
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

double findDistance(OPoint, OPoint);
double findAngle(OPoint, OPoint);
double smallestAngle(double, double);
double angleToInches(double);

#define AUTO_INTAKE_VEL 200
#define AUTO_INDEXER_VEL 200
#define AUTO_SHOOTER_VEL 200

#endif //_AUTONOMOUS_H_
