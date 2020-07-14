#include "main.h"
#include "opcontrol.h"
#include "motors.h"
#include "utilities.h"

void splitArcade(void*) {
  // std::pair<int, int> stuff = *static_cast<std::pair<int, int>*> (driveValues);
  double power, turn, left, right;
  while (true) {
    //Standard split arcade drive implementation
    power = master.get_analog(ANALOG_LEFT_Y);
    turn = master.get_analog(ANALOG_RIGHT_X);

    //left and right represent percentages of max speed
    left = (power + turn) * 100.0 / 127;
    left = std::max(-100.0, std::min(left, 100.0));
    right = (power - turn) * 100.0 / 127;
    right = std::max(-100.0, std::min(right, 100.0));

    //Each of left and right are now mapped to a value between
    //0 and 200 using a custom exponential curve to ensure precise
    //micro movement (especially turns) while maintaining high max speed.
    //See bit.ly/vexjoystickmap for a graph of input vs. output.

    left = sgn(left) * (std::floor(1.045 * std::exp(std::fabs(left) / 19.0)) - 1);
    right = sgn(right) * (std::floor(1.045 * std::exp(std::fabs(right) / 19.0)) - 1);

    if ((abs(left) > 10) || (abs(right) > 10)) {
      leftFront.move_velocity((int) left);
      leftBack.move_velocity((int) left);
      rightFront.move_velocity((int) right);
      rightBack.move_velocity((int) right);
    }
    else {
      //don't move if there's very little input
      leftFront.move_voltage(0);
      leftBack.move_voltage(0);
      rightFront.move_voltage(0);
      rightBack.move_voltage(0);
    }
    pros::delay(20);
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
        shooter.move_voltage(0);
      }
    }
    else {
      if ((master.get_digital(DIGITAL_R1)) || (master.get_digital(DIGITAL_A))) {
        shooter.move_voltage(SHOOTER_SPEED);
      }
      else {
        shooter.move_voltage(0);
      }
    }
    pros::delay(20);
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
        leftIntake.move_voltage(0);
        rightIntake.move_voltage(0);
      }
    }
    else {
      if ((master.get_digital(DIGITAL_L1)) || (master.get_digital(DIGITAL_A))) {
        leftIntake.move_voltage(INTAKE_SPEED);
        rightIntake.move_voltage(-INTAKE_SPEED);
      }
      else {
        leftIntake.move_voltage(0);
        rightIntake.move_voltage(0);
      }
    }
    pros::delay(20);
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
        indexer.move_voltage(0);
      }
    }
    else {
      if ((master.get_digital(DIGITAL_R2)) || (master.get_digital(DIGITAL_A))) {
        indexer.move_voltage(INDEXER_SPEED);
      }
      else {
        indexer.move_voltage(0);
      }
    }
    pros::delay(20);
  }
}
