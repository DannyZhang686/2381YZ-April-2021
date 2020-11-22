#include "main.h"
#include "autonomous.h"

//Mutexes
pros::Mutex arrivedAtFirstGoal, doneWithFirstGoal, doneFirstDiscard;

//Autonomous routines
void movementOne(void*) {
  arrivedAtFirstGoal.take(0);
  robotPos.x = 36;
  robotPos.y = 12;
  robotPos.angle = 0;
  pros::delay(250);
  moveShort(36, 24, 2, true);
  pros::delay(250);
  turnToPoint(14, 15, 0.1);
  pros::delay(250);
  moveShort(14, 15, 1.5, true);
  arrivedAtFirstGoal.give();
  // turnToPoint(13, 14.5, 0.1);
  // pros::delay(250);
  // moveShort(13, 14.5, 1.5, true);
  doneWithFirstGoal.take(INT_MAX);
  // pros::delay(1500); //add time
  // turnToOppositePoint(36, 24, 0.1);
  moveShort(36, 24, 2, false);
  pros::delay(250);
  turnToPoint(11, 38, 0.1);
  pros::delay(250);
  doneFirstDiscard.give();
  moveShort(11, 38, 1.5, true);
  pros::delay(250);
  turnToPoint(33, 78, 0.1);
  pros::delay(250);
  moveShort(33, 78, 2, true);
  pros::delay(250);
  turnToPoint(9, 72, 0.1);
  pros::delay(250);

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
  doneWithFirstGoal.take(0);
  pros::delay(250);
  intakeNoShoot();
  arrivedAtFirstGoal.take(INT_MAX);
  intakeShoot(1, 2);
  doneWithFirstGoal.give();
  pros::delay(300);
  discard();
  doneFirstDiscard.take(INT_MAX);
  intakeNoShoot();
  stopMotors();
}

void movementTwo(void*) {}

void snailTwo(void*) {}
