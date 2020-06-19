#include "main.h"
#include "opcontrol.h"
#include "motors.h"

void splitArcade(void*) {
  // std::pair<int, int> stuff = *static_cast<std::pair<int, int>*> (driveValues);
  int power, turn, left, right;
  //This is a little bugged for some reason, fix next time
  while (true) {
    //Standard split arcade drive implementation
    power = master.get_analog(ANALOG_LEFT_Y);
    turn = master.get_analog(ANALOG_RIGHT_X);
    left = power + turn;
    right = power - turn;
    leftFront.move_velocity(left);
    leftBack.move_velocity(left);
    rightFront.move_velocity(right);
    rightBack.move_velocity(right);
  }
}

//For these robot functions, DIGITAL_L2 is used as a "shift" key;
//when it is pressed along with another button, a different
//(usually opposite) command is given.

//This increases the number of different things the driver can do
//using the four buttons on the controller that are easiest to reach.

void flywheelSpin(void*) {
  //DIGITAL_R1 is the button assigned to the flywheel
  while (true) {
    if (master.get_digital(DIGITAL_L2)) {
      if (master.get_digital(DIGITAL_R1)) {
        flywheel.move_velocity(-FLYWHEEL_SPEED);
      }
      else {
        flywheel.move_velocity(0);
      }
    }
    else {
      if (master.get_digital(DIGITAL_R1)) {
        flywheel.move_velocity(FLYWHEEL_SPEED);
      }
      else {
        flywheel.move_velocity(0);
      }
    }
  }
}

void intakeSpin(void*) {
  //DIGITAL_L2 is the button assigned to the intakes
  while (true) {
    if (master.get_digital(DIGITAL_L2)) {
      if (master.get_digital(DIGITAL_L1)) {
        leftIntake.move_velocity(-INTAKE_SPEED);
        rightIntake.move_velocity(INTAKE_SPEED);
      }
      else {
        leftIntake.move_velocity(0);
        rightIntake.move_velocity(0);
      }
    }
    else {
      if (master.get_digital(DIGITAL_L1)) {
        leftIntake.move_velocity(INTAKE_SPEED);
        rightIntake.move_velocity(-INTAKE_SPEED);
      }
      else {
        leftIntake.move_velocity(0);
        rightIntake.move_velocity(0);
      }
    }
  }
}

void indexerSpin(void*) {
  //DIGITAL_R2 is the button assigned to the indexer
  while (true) {
    if (master.get_digital(DIGITAL_L2)) {
      if (master.get_digital(DIGITAL_R2)) {
        indexer.move_velocity(-INDEXER_SPEED);
      }
      else {
        indexer.move_velocity(0);
      }
    }
    else {
      if (master.get_digital(DIGITAL_R2)) {
        indexer.move_velocity(INDEXER_SPEED);
      }
      else {
        indexer.move_velocity(0);
      }
    }
  }
}
