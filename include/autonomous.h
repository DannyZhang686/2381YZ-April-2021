#include "main.h"

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

//General function declarations
void trackPosition(void*);
void moveShort(double, double);
void moveLong(double, double);
void turnToFace(double);

//Auton routines
void movementOne(void*);
void snailOne(void*);
void movementTwo(void*);
void snailTwo(void*);

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
  void resetValues() {
    left = right = back = 0;
  }
  EncoderVal(): left(0), right(0), back(0) {}
} EncoderVal;

//Change in values struct
//Units: inches
typedef struct DeltaVal {
  double left, right, back;
  void resetValues() {
    left = right = back = 0;
  }
  DeltaVal(): left(0), right(0), back(0) {}
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
extern pros::Mutex driveCommand;
extern pros::Mutex pdGetOutput;
extern int autonPick;

#endif //_AUTONOMOUS_H_
