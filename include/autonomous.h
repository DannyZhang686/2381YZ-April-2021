#include "main.h"

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

//auton function declarations
void trackPosition(void*);
void moveShort(double, double);
void moveLong(double, double);
void turnToFace(double);

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

//Other extern variables
extern int autonPick;

#endif //_AUTONOMOUS_H_
