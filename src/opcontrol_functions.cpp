#include "main.h"
#include "opcontrol.h"
#include "motors.h"
#include "utilities.h"
#include "control/motor_controller.hpp"
#include "control/pid.hpp"
//For these robot functions, DIGITAL_L2 is used as a "shift" key;
//when it is pressed along with another button, a different
//(usually opposite) command is given.

//This increases the number of different things the driver can do
//using the four buttons on the controller that are easiest to reach.

void shooterSpin(void*) {
  //DIGITAL_R1 is the button assigned to the flywheel
  while (true) {
    if ((master.get_digital(DIGITAL_R1)) || (master.get_digital(DIGITAL_A))) {
      shooter.move_voltage(SHOOTER_SPEED);
    }
    else if (master.get_digital(DIGITAL_X)) {
      shooter.move_voltage(-SHOOTER_SPEED);
    }
    else {
      shooter.move_voltage(0);
    }
    pros::delay(20);
  }
}

void intakeSpin(void*) {
  //DIGITAL_L2 is the button assigned to the intakes
  while (true) {
    if (master.get_digital(DIGITAL_L2)) {
      leftIntake.move_voltage(-INTAKE_SPEED);
      rightIntake.move_voltage(-INTAKE_SPEED);
    }
    else if ((master.get_digital(DIGITAL_L1)) || (master.get_digital(DIGITAL_A)) || (master.get_digital(DIGITAL_X))) {
      leftIntake.move_voltage(INTAKE_SPEED);
      rightIntake.move_voltage(INTAKE_SPEED);
    }
    else {
      leftIntake.move_voltage(0);
      rightIntake.move_voltage(0);
    }
    pros::delay(20);
  }
}

void indexerSpin(void*) {
  //DIGITAL_R2 is the button assigned to the indexer
  while (true) {
    if ((master.get_digital(DIGITAL_R2)) || (master.get_digital(DIGITAL_A)) || (master.get_digital(DIGITAL_X))) {
      indexer.move_voltage(INDEXER_SPEED);
    }
    else {
      indexer.move_voltage(0);
    }
    pros::delay(20);
  }
}
