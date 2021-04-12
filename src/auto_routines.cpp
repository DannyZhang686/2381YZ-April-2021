#include "main.h"
#include "autonomous.h"
#include "motors.h"
#include "pid.h"

//Mutexes
pros::Mutex arrivedAtGoal[9];
pros::Mutex doneWithGoal[9];
pros::Mutex doneDiscard;

//Autonomous routines
void movementOne(void*) {
  for (int i = 0; i < 9; i++) {
    arrivedAtGoal[i].take(0);
  }
  robotPos.x = 36;
  robotPos.y = 12;
  robotPos.angle = 0;
  movePurePursuit(36, 30, 2.5, 8, true); //36, 36
  turnToPoint(12, 15, 0.1); //13, 16
  movePurePursuit(12, 15, 3, 8, true);
  approachGoal(50, 650);
  arrivedAtGoal[0].give();
  // pros::delay(1000);

  doneWithGoal[0].take(INT_MAX);
  backAway(70, 500);
  pros::delay(600);
  // turnToPoint(11, 39.5, 0.05);
  // movePurePursuit(11, 39.5, 2.5, 8, true); //Getting the ball on the side wall
  // backAway(70, 500);
  turnToPoint(28.5, 74, 0.1);
  movePurePursuit(28.5, 74, 2.5, 8, true);
  turnToPoint(17, 75, 0.1);
  movePurePursuit(17, 75, 2.5, 8, true);
  approachGoal(50, 650);
  arrivedAtGoal[1].give();
  // pros::delay(1000);

  doneWithGoal[1].take(INT_MAX);
  backAway(65, 650);
  pros::delay(600);
  turnToPoint(60, 75, 0.1);
  movePurePursuit(60, 75, 2.5, 8, true);
  for (int i = 0; i < 2; i++) {
    approachGoal(130, 650);
    pros::delay(300);
    backAway(130, 400);
    pros::delay(400);
  }
  approachGoal(100, 1000);
  pros::delay(800);
  arrivedAtGoal[2].give();
  // pros::delay(1000);

  doneWithGoal[2].take(INT_MAX);

  //Code is UNTESTED below this point
  // backAway(130, 400);
  // turnToPoint(36, 108, 0.1); //Coordinates of ball
  // movePurePursuit(36, 108, 2.5, 8, true);
  // turnToPoint(12, 135, 0.1); //Coordinates of goal
  // movePurePursuit(12, 135, 2.5, 8, true);
  // approachGoal(50, 650);
  // arrivedAtGoal[3].give();
  // pros::delay(1000);

  // doneWithGoal[3].take(INT_MAX);
  // backAway(65, 650);
  // turnToPoint(72, 96, 0.1); //Coordinates of ball
  // movePurePursuit(72, 96, 2.5, 8, true);
  // turnToPoint(72, 127, 0.1); //Coordinates of goal
  // movePurePursuit(72, 127, 2.5, 8, true);
  // approachGoal(50, 650);
  // arrivedAtGoal[4].give();
  // // pros::delay(1000);
  //
  // doneWithGoal[4].take(INT_MAX);
  // backAway(65, 650);
  // turnToPoint(108, 108, 0.1); //Coordinates of ball
  // movePurePursuit(108, 108, 2.5, 8, true);
  // turnToPoint(132, 129, 0.1); //Coordinates of goal
  // movePurePursuit(132, 129, 2.5, 8, true);
  // approachGoal(50, 650);
  // arrivedAtGoal[5].give();
}

void snailOne(void*) {
  for (int i = 0; i < 9; i++) {
    doneWithGoal[i].take(0);
  }
  pros::delay(250);
  intakeNoShoot(200);
  arrivedAtGoal[0].take(INT_MAX);

  stopMotors();
  pros::delay(100);
  intakeShoot(0, 1); // intakeShoot(2, 2); with descoring
  stopMotors();
  pros::delay(150);
  doneWithGoal[0].give();
  pros::delay(300);
  discard();
  pros::delay(600);

  intakeNoShoot(200);
  arrivedAtGoal[1].take(INT_MAX);

  stopMotors();
  pros::delay(200);
  intakeShoot(0, 2); // intakeShoot(1, 1); with descoring
  stopMotors();
  pros::delay(150);
  doneWithGoal[1].give();
  pros::delay(250);
  discard();
  pros::delay(600);

  intakeNoShoot(200);
  arrivedAtGoal[2].take(INT_MAX);

  stopMotors();
  pros::delay(200);
  intakeShoot(0, 1);
  stopMotors();
  pros::delay(150);
  doneWithGoal[2].give();
  pros::delay(500);

  // intakeNoShoot(200);
  // arrivedAtGoal[3].take(INT_MAX);
  //
  // stopMotors();
  // pros::delay(200);
  // intakeShoot(0, 1);
  // stopMotors();
  // pros::delay(150);
  // doneWithGoal[3].give();
  // pros::delay(500);

  // intakeNoShoot(200);
  // arrivedAtGoal[4].take(INT_MAX);
  //
  // stopMotors();
  // pros::delay(200);
  // intakeShoot(0, 1);
  // stopMotors();
  // pros::delay(150);
  // doneWithGoal[4].give();

  stopMotors();

  // s__t(2, "1");
  // intakeNoShoot(200);
  // s__t(2, "2");
  // pros::delay(2000);
  // s__t(2, "3");
  // stopMotors();
  // s__t(2, "4");
  // pros::delay(2000);
  // s__t(2, "5");
  // intakeShoot(2, 2);
  // s__t(2, "6");
  // stopMotors();
  // s__t(2, "7");
}

void movementTwo(void*) {}

void snailTwo(void*) {}
