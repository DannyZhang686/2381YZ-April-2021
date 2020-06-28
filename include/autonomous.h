#include "main.h"

#ifndef _PROS_AUTONOMOUS_H_
#define _PROS_AUTONOMOUS_H_

//auton function declarations
void trackPosition(void*);
void moveShort(double, double);
void moveLong(double, double);
void turnToFace(double);

//Position tracking struct and variable
struct Position {
  double x;
  double y;
  double angle;
  void reset() {
    x = y = angle = 0;
  }
} extern robotPos;

//extern variables
extern int autonPick;

#endif //_PROS_AUTONOMOUS_H_
