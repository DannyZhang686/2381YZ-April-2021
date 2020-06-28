#include "main.h"
#include "autonomous.h"
#include "motors.h"

void trackPosition(void*) {
  //tracking algorithms
}

void moveShort(double targetX, double targetY) {
  //moves with a full PID
}

void moveLong(double targetX, double targetY) {
  //moves with position tracking and ends with PID
}

void turnToFace(double angle) {
  //turns with full PID
}
