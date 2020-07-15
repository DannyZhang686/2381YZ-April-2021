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
struct Position {
  double x, y, angle;
  void resetPos() {
    x = y = angle = 0;
  }
  //Constructor to default all members to 0
  Position(): x(0), y(0), angle(0) {}
} extern robotPos;

//Encoder values struct
//Units: encoder units
struct EncoderVal {
  int left, right, back;
  void resetValues() {
    left = right = back = 0;
  }
  EncoderVal(): left(0), right(0), back(0) {}
} extern currentVal, lastVal;

//Change in values struct
//Units: inches
struct DeltaVal {
  double left, right, back;
  void resetValues() {
    left = right = back = 0;
  }
  DeltaVal(): left(0), right(0), back(0) {}
} extern deltaVal;

//Struct defining a point on the field
//Units: inches
struct Point {
  double x, y;
  void resetValues() {
    x = y = 0;
  }
  void setValues(int newX, int newY) {
    x = newX;
    y = newY;
  }
  Point(): x(0), y(0) {}
  Point(int initX, int initY): x(initX), y(initY) {}
};

//Extern variables
extern pros::Mutex driveCommand;
extern int autonPick;

#endif //_AUTONOMOUS_H_
