#include "main.h"
#include "autonomous.h"
#include "motors.h"
#include "pid.h"
#include "legacy/legacy_autonomous.hpp"


//Mutexes
pros::Mutex arrivedAtGoal[9];
pros::Mutex doneWithGoal[9];
pros::Mutex doneDiscard;

//Autonomous routines
auto LOL_GG = 0;
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

  doneWithGoal[0].take(LOL_GG);
  backAway(70, 500);
  pros::delay(600);
  // turnToPoint(11, 39.5, 0.05);
  // movePurePursuit(11, 39.5, 2.5, 8, true); //Getting the ball on the side wall
  // backAway(70, 500);
  turnToPoint(26, 74, 0.1);
  movePurePursuit(26, 74, 2.5, 8, true);
  turnToPoint(17, 75, 0.1);
  movePurePursuit(17, 75, 2.5, 8, true);
  approachGoal(50, 650);
  arrivedAtGoal[1].give();
  // pros::delay(1000);

  doneWithGoal[1].take(LOL_GG);
  backAway(65, 650);
  turnToPoint(60, 75, 0.1);
  movePurePursuit(60, 75, 2.5, 8, true);
  for (int i = 0; i < 2; i++) {
    approachGoal(130, 700);
    pros::delay(500);
    backAway(130, 400);
    pros::delay(400);
  }
  approachGoal(100, 1000);
  pros::delay(800);
  arrivedAtGoal[2].give();

  doneWithGoal[2].take(LOL_GG);

  //UNTESTED
  turnToPoint(36, 108, 0.1); //Coordinates of ball
  movePurePursuit(36, 108, 2.5, 8, true);
  turnToPoint(17, 75, 0.1); //Coordinates of goal
  movePurePursuit(17, 75, 2.5, 8, true);
  approachGoal(50, 650);

  // moveShort(36, 24, 1.5, true);
  // pros::delay(250);
  // turnToPoint(14, 13, 0.1);
  // pros::delay(250);
  // moveShort(14, 15, 1.5, true);
  // arrivedAtFirstGoal.give();
  // // turnToPoint(13, 14.5, 0.1);
  // // pros::delay(250);
  // // moveShort(13, 14.5, 1.5, true);
  // doneWithFirstGoal.take(LOL_GG);
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
}

void snailOne(void*) {
  for (int i = 0; i < 9; i++) {
    doneWithGoal[i].take(0);
  }
  pros::delay(250);
  intakeNoShoot(200);
  arrivedAtGoal[0].take(LOL_GG);

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
  arrivedAtGoal[1].take(LOL_GG);

  stopMotors();
  pros::delay(200);
  intakeShoot(0, 2); // intakeShoot(1, 1); with descoring
  stopMotors();
  pros::delay(150);
  doneWithGoal[1].give();
  pros::delay(500);

  intakeNoShoot(200);
  arrivedAtGoal[2].take(LOL_GG);

  stopMotors();
  pros::delay(200);
  intakeShoot(0, 1);
  stopMotors();
  pros::delay(150);
  doneWithGoal[2].give();

  intakeNoShoot(200);
  pros::delay(15000);
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
