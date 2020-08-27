#include "main.h"
#include "opcontrol.h"
#include "motors.h"
#include "utilities.h"

#define DEAD_ZONE 20 //The joystick deadzone in percent of half the joystick range
#define MAX_DELTA_SPEED 500 //The maximum permitted change in target speed in the drive in the equivalent of mV/20 ms

void splitArcade(void*) {
  double power, turn, left, right; //Part of the split arcade implementation
  double actualLeftSpeed, actualRightSpeed; //The actual speed for the left and right drive in the equivalent mV value
  while (true) {
    //Standard split arcade drive implementation
    power = master.get_analog(ANALOG_LEFT_Y);
    turn = master.get_analog(ANALOG_RIGHT_X);

    //left and right represent percentages of max speed
    left = (power + turn) * 100.0 / 127;
    left = std::max(-100.0, std::min(left, 100.0));
    right = (power - turn) * 100.0 / 127;
    right = std::max(-100.0, std::min(right, 100.0));

    if ((abs(left) > DEAD_ZONE) || (abs(right) > DEAD_ZONE)) {
      //Each of left and right are now mapped to a value between
      //0 and 12000 using a custom exponential curve to ensure precise
      //micro movement (especially turns) while maintaining high max speed.
      //See http://bit.ly/voltagecurve for a graph of input vs. output.

      // left = sgn(left) * (floor(62.7 * exp(fabs(left) / 19.0)) - 62);
      // right = sgn(right) * (floor(62.7 * exp(fabs(right) / 19.0)) - 62);

      left = sgn(left) * (floor(52.5 * exp(fabs(left) / 19.0)) + 1862);
      right = sgn(right) * (floor(52.5 * exp(fabs(right) / 19.0)) + 1862);
    }
    else {
      //Move at voltage 0 if there's very little or no input
      left = right = 0;
    }

    //Limit the maximum change in motor speed to prevent tipping and chain snapping
    actualLeftSpeed = (leftFront.get_actual_velocity() + leftBack.get_actual_velocity()) * 30; //Multiplied by 0.5 * 60 for average and conversion from rpm (0-200) to equivalent mV (0-12000)
    actualRightSpeed = (rightFront.get_actual_velocity() + rightBack.get_actual_velocity()) * 30;
    if (abs(left - actualLeftSpeed) > MAX_DELTA_SPEED) {
      //Outside the allowable range; set speed as appropriate
      if (left > actualLeftSpeed) {
        left = actualLeftSpeed + MAX_DELTA_SPEED;
      }
      else {
        left = actualLeftSpeed - MAX_DELTA_SPEED;
      }
    }
    if (abs(right - actualRightSpeed) > MAX_DELTA_SPEED) {
      if (right > actualRightSpeed) {
        right = actualRightSpeed + MAX_DELTA_SPEED;
      }
      else {
        right = actualRightSpeed - MAX_DELTA_SPEED;
      }
    }
    leftFront.move_voltage((int) left);
    leftBack.move_voltage((int) left);
    rightFront.move_voltage((int) right);
    rightBack.move_voltage((int) right);
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
      else if (master.get_digital(DIGITAL_B)) {
        shooter.move_voltage(SHOOTER_SPEED);
      }
      else {
        shooter.move_voltage(0);
      }
    }
    else {
      if ((master.get_digital(DIGITAL_R1)) || (master.get_digital(DIGITAL_A))) {
        shooter.move_voltage(SHOOTER_SPEED);
      }
      else if (master.get_digital(DIGITAL_B)) {
        shooter.move_voltage(-SHOOTER_SPEED);
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
      if ((master.get_digital(DIGITAL_L1)) || (master.get_digital(DIGITAL_A)) || (master.get_digital(DIGITAL_B))) {
        leftIntake.move_voltage(-INTAKE_SPEED);
        rightIntake.move_voltage(-INTAKE_SPEED);
      }
      else {
        leftIntake.move_voltage(0);
        rightIntake.move_voltage(0);
      }
    }
    else {
      if ((master.get_digital(DIGITAL_L1)) || (master.get_digital(DIGITAL_A)) || (master.get_digital(DIGITAL_B))) {
        leftIntake.move_voltage(INTAKE_SPEED);
        rightIntake.move_voltage(INTAKE_SPEED);
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
      if ((master.get_digital(DIGITAL_R2)) || (master.get_digital(DIGITAL_A)) || (master.get_digital(DIGITAL_B))) {
        indexer.move_voltage(-INDEXER_SPEED);
      }
      else {
        indexer.move_voltage(0);
      }
    }
    else {
      if ((master.get_digital(DIGITAL_R2)) || (master.get_digital(DIGITAL_A)) || (master.get_digital(DIGITAL_B))) {
        indexer.move_voltage(INDEXER_SPEED);
      }
      else {
        indexer.move_voltage(0);
      }
    }
    pros::delay(20);
  }
}
