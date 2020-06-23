#include "main.h"
#include "opcontrol.h"
#include "motors.h"

void splitArcade(void*) {
  // std::pair<int, int> stuff = *static_cast<std::pair<int, int>*> (driveValues);
  int power, turn, left, right;
  while (true) {
    //Standard split arcade drive implementation
    power = master.get_analog(ANALOG_LEFT_Y);
    turn = master.get_analog(ANALOG_RIGHT_X);
    left = power + turn;
    right = power - turn;
    if ((abs(left) > 10) || (abs(right) > 10)) {
      //don't move if there's very little input
      leftFront.move_velocity(left);
      leftBack.move_velocity(left);
      rightFront.move_velocity(right);
      rightBack.move_velocity(right);
    }
    else {
      leftFront.move_velocity(0);
      leftBack.move_velocity(0);
      rightFront.move_velocity(0);
      rightBack.move_velocity(0);
    }
    pros::delay(10);
  }
}

//For these robot functions, DIGITAL_L2 is used as a "shift" key;
//when it is pressed along with another button, a different
//(usually opposite) command is given.

//This increases the number of different things the driver can do
//using the four buttons on the controller that are easiest to reach.

void shooterSpin(void*) {
  //DIGITAL_R1 is the button assigned to the flywheel
  while (true) {
    if (master.get_digital(DIGITAL_L2)) {
      if ((master.get_digital(DIGITAL_R1)) || (master.get_digital(DIGITAL_A))) {
        shooter.move_voltage(-SHOOTER_SPEED);
      }
      else {
        shooter.move_velocity(0);
      }
    }
    else {
      if ((master.get_digital(DIGITAL_R1)) || (master.get_digital(DIGITAL_A))) {
        shooter.move_voltage(SHOOTER_SPEED);
      }
      else {
        shooter.move_velocity(0);
      }
    }
    pros::delay(10);
  }
}

void intakeSpin(void*) {
  //DIGITAL_L2 is the button assigned to the intakes
  while (true) {
    if (master.get_digital(DIGITAL_L2)) {
      if ((master.get_digital(DIGITAL_L1)) || (master.get_digital(DIGITAL_A))) {
        leftIntake.move_voltage(-INTAKE_SPEED);
        rightIntake.move_voltage(INTAKE_SPEED);
      }
      else {
        leftIntake.move_velocity(0);
        rightIntake.move_velocity(0);
      }
    }
    else {
      if ((master.get_digital(DIGITAL_L1)) || (master.get_digital(DIGITAL_A))) {
        leftIntake.move_voltage(INTAKE_SPEED);
        rightIntake.move_voltage(-INTAKE_SPEED);
      }
      else {
        leftIntake.move_velocity(0);
        rightIntake.move_velocity(0);
      }
    }
    pros::delay(10);
  }
}

void indexerSpin(void*) {
  //DIGITAL_R2 is the button assigned to the indexer
  while (true) {
    if (master.get_digital(DIGITAL_L2)) {
      if ((master.get_digital(DIGITAL_R2)) || (master.get_digital(DIGITAL_A))) {
        indexer.move_voltage(-INDEXER_SPEED);
      }
      else {
        indexer.move_velocity(0);
      }
    }
    else {
      if ((master.get_digital(DIGITAL_R2)) || (master.get_digital(DIGITAL_A))) {
        indexer.move_voltage(INDEXER_SPEED);
      }
      else {
        indexer.move_velocity(0);
      }
    }
    pros::delay(10);
  }
}
