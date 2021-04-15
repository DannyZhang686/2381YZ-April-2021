#include "main.h"
#include "opcontrol.h"
#include "motors.h"
#include "utilities.h"
#include "control/motor_controller.hpp"
#include "control/pid.hpp"
#include "globals.hpp"
//For these robot functions, DIGITAL_L2 is used as a "shift" key;
//when it is pressed along with another button, a different
//(usually opposite) command is given.

//This increases the number of different things the driver can do
//using the four buttons on the controller that are easiest to reach.

bool shooterToggle = true;
double prevNumBalls = 0;

void shooterSpin(void *)
{
  //DIGITAL_R1 is the button assigned to the flywheel
  while (true)
  {

    if ((floor(numBallsShot) == numBallsShot) && (numBallsShot != prevNumBalls))
    {
      shooterToggle = false;
    }
    prevNumBalls = numBallsShot;

    if ((master.get_digital_new_press(DIGITAL_A)) || (master.get_digital_new_press(DIGITAL_R1)))
    {
      shooterToggle = true;
    }

    if ((master.get_digital(DIGITAL_R1)) || ((master.get_digital(DIGITAL_A)) && (shooterToggle)))
    {
      shooter.move_voltage(SHOOTER_SPEED);
    }
    else if ((master.get_digital(DIGITAL_L1)) || (master.get_digital(DIGITAL_Y)))
    {
      shooter.move_voltage(-SHOOTER_SPEED);
    }
    else
    {
      shooter.move_voltage(0);
    }
    pros::delay(20);
  }
}

void intakeSpin(void *)
{
  //DIGITAL_L2 is the button assigned to the intakes
  while (true)
  {
    if ((master.get_digital(DIGITAL_L2)) || (master.get_digital(DIGITAL_Y)))
    {
      leftIntake.move_voltage(-INTAKE_SPEED);
      rightIntake.move_voltage(-INTAKE_SPEED);
    }
    else if ((master.get_digital(DIGITAL_L1)) || (master.get_digital(DIGITAL_A)) || (master.get_digital(DIGITAL_X)))
    {
      //Forwards
      leftIntake.move_voltage(INTAKE_SPEED);
      rightIntake.move_voltage(INTAKE_SPEED);
    }
    else
    {
      leftIntake.move_voltage(0);
      rightIntake.move_voltage(0);
    }
    pros::delay(20);
  }
}

void controllerIndexerSpin()
{
  if (master.get_digital(DIGITAL_Y))
  {
    indexerSetpoint = (-INDEXER_SPEED);
  }
  else if ((master.get_digital(DIGITAL_R2)) || (master.get_digital(DIGITAL_A)) || (master.get_digital(DIGITAL_L1)))
  {
    indexerSetpoint = (INDEXER_SPEED);
  }
  else
  {
    indexerSetpoint = (0);
  }
  s__t(0, "indexer time: " + t__s(pros::millis()) + " : " + t__s(master.get_digital(DIGITAL_Y)));
}

void indexerSpin(void *)
{
  //DIGITAL_R2 is the button assigned to the indexer
  while (true)
  {
    if(!indexerOn)
    {

    }
    else if (STOP)
    {
      indexer.move_voltage(0);
    }
    else
    {
      indexer.move_voltage(indexerSetpoint);
    }
    pros::delay(20);
  }
}
