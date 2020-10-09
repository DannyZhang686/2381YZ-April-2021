#include "main.h"
#include "autonomous.h"

//Autonomous routines

void movementOne(void*) {
  // moveShort(0, 20, 3, true);
  // pros::delay(5000);
  turnToFace(1.57, 0.1);
}

void snailOne(void*) {
  intakeNoShoot();
}

void movementTwo(void*) {}

void snailTwo(void*) {}
