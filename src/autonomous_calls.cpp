#include "main.h"
#include "autonomous.h"
#include "motors.h"

void trackPosition(void*) {
  //tracking and stuff
}

void moveForward(int speed) {
  //moves forward
  leftFront.move_velocity(speed);
}
