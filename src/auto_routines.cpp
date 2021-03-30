#include "main.h"
#include "autonomous.h"
#include "motors.h"
#include "pid.h"

//Mutexes
pros::Mutex arrivedAtFirstGoal, doneWithFirstGoal, doneFirstDiscard;

//Autonomous routines
void movementOne(void*) {
  // double leftkP = 1, leftkD = 3; //speculative tuned values
  // double rightkP = 0, rightkD = 0;
  // Pd leftStraight(leftkP, leftkD);
  // Pd leftTurn(leftkP, leftkD);
  // Pd rightStraight(rightkP, rightkD);
  // Pd rightTurn(rightkP, rightkD);
  // while (true) {
  //   double left = leftStraight.getOutput(rightFront.get_position(), 500);
  //   s__t(0, t__s(left) + " " + t__s(rightFront.get_position()));
  //   // double right = rightStraight.getOutput(rightFront.get_position(), 1000);
  //   leftFront.move(left); //Call to PID to find velocity
  //   leftBack.move(left);
  //   rightFront.move(left);
  //   rightBack.move(left);
  //   pros::delay(20);
  // }
  // arrivedAtFirstGoal.take(0);
  robotPos.x = 36;
  robotPos.y = 12;
  robotPos.angle = 0;
  movePurePursuit(36, 30, 2.5, 8, true); //36, 36
  turnToPoint(12, 15, 0.1); //13, 16
  movePurePursuit(12, 15, 3, 8, true);
  approachGoal(50, 650);
  // arrivedAtFirstGoal.give();
  // doneWithFirstGoal.take(INT_MAX);
  backAway(70, 500);
  pros::delay(150);
  // turnToPoint(11, 39.5, 0.05);
  // movePurePursuit(11, 39.5, 2.5, 8, true); //11, 39.5
  // backAway(70, 500);
  turnToPoint(26, 74, 0.1);
  movePurePursuit(26, 74, 2.5, 8, true);
  turnToPoint(17, 75, 0.1);
  movePurePursuit(17, 75, 2.5, 8, true);
  approachGoal(50, 650);
  pros::delay(1000);
  backAway(65, 650);
  turnToPoint(60, 75, 0.1);
  movePurePursuit(60, 75, 2.5, 8, true);
  approachGoal(80, 1000);
  pros::delay(1000);
  backAway(65, 650);

  // moveShort(36, 24, 1.5, true);
  // pros::delay(250);
  // turnToPoint(14, 13, 0.1);
  // pros::delay(250);
  // moveShort(14, 15, 1.5, true);
  // arrivedAtFirstGoal.give();
  // // turnToPoint(13, 14.5, 0.1);
  // // pros::delay(250);
  // // moveShort(13, 14.5, 1.5, true);
  // doneWithFirstGoal.take(INT_MAX);
  // // pros::delay(1500); //add time
  // // turnToOppositePoint(36, 24, 0.1);
  // moveShort(36, 24, 2, false);
  // pros::delay(250);
  // turnToPoint(11, 38, 0.1);
  // pros::delay(250);
  // doneFirstDiscard.give();
  // moveShort(11, 38, 1.5, true);
  // pros::delay(250);
  // turnToPoint(33, 78, 0.1);
  // pros::delay(250);
  // moveShort(33, 78, 2, true);
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
  doneWithFirstGoal.take(0);
  pros::delay(250);
  intakeNoShoot(200);
  arrivedAtFirstGoal.take(INT_MAX);
  intakeShoot(1, 2);
  doneWithFirstGoal.give();
  pros::delay(300);
  discard();
  pros::delay(300);
  // doneFirstDiscard.take(INT_MAX);
  // intakeNoShoot(200);
  stopMotors();
  pros::delay(200000);
}

void movementTwo(void*) {}

void snailTwo(void*) {}
