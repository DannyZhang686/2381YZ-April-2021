#include "main.h"
#include "autonomous.h"

//Autonomous routines

void movementOne(void*) {
  robotPos.x = 36;
  robotPos.y = 12;
  robotPos.angle = 0;
  moveShort(36, 24, 2, true);
  pros::delay(250);
  turnToPoint(11, 11, 0.1);
  pros::delay(250);
  // turnToPoint(-11, -11, 0.05);
  // pros::delay(250);
  moveShort(11, 11, 1.5, true);
  pros::delay(250); //add time
  moveShort(36, 24, 2, false);
  pros::delay(250);
  // turnToPoint(6.5, 36, 0.1);
  // pros::delay(250);
  // moveShort(6.5, 36, 1.5, true);
  // pros::delay(250);
  // moveShort(24, 72, 2, true);
  // pros::delay(250);
  // turnToPoint(9, 72, 0.1);
  // pros::delay(250);
  // moveShort(9, 72, 1.5, true);
  // pros::delay(3000);
  // moveShort(24, 72, 1.5, false);
  // pros::delay(250);
  // turnToPoint(72, 72, 0.1);
  // pros::delay(250);
  // moveShort(72, 72, 1.5, true);
  // pros::delay(3000);
}

void snailOne(void*) {
  intakeNoShoot();
}

void movementTwo(void*) {}

void snailTwo(void*) {}
