#include "main.h"
#include "opcontrol.h"
#include "motors.h"
#include "utilities.h"
#include "control/motor_controller.hpp"
#include "control/pid.hpp"
#include "globals.hpp"
#include "legacy/legacy_autonomous.hpp"
#include "autonomous.h"
#include "control/pid.hpp"

//For these robot functions, DIGITAL_L2 is used as a "shift" key;
//when it is pressed along with another button, a different
//(usually opposite) command is given.

//This increases the number of different things the driver can do
//using the four buttons on the controller that are easiest to reach.

bool shooterToggle = true;
double prevNumBalls = 0;

const void controllerShooterSpin(void)
  {

  if (((master.get_digital(DIGITAL_R1)) || (master.get_digital(DIGITAL_A))) && (shooterToggle))
    {
    shooterSetpoint = (SHOOTER_SPEED);
    }
  else if ((master.get_digital(DIGITAL_L1)) || (master.get_digital(DIGITAL_Y)))
    {
    shooterSetpoint = (-SHOOTER_SPEED);
    }
  else
    {
    shooterSetpoint = (0);
    }
  }

PID shooterPID = PID(1, 0, 0);

void shooterSpin(void*)
  {
  //DIGITAL_R1 is the button assigned to the flywheel
  while (true)
    {
    if (shooterOn)
      {
      if (STOP)
        {
        shooterSetpoint = 0;
        }

      if (shooterSetpoint == 0)
        {
        double shooterVoltage = shooterPID.Update(0, shooter.get_actual_velocity());
        shooter.move(shooterVoltage);
        }
      else
        {
        shooter.move_voltage(shooterSetpoint);
        }
      }
    pros::delay(DELAY_INTERVAL);
    }
  }

const void controllerIntakeSpin(void)
  {
  if ((master.get_digital(DIGITAL_L2)) || (master.get_digital(DIGITAL_Y)))
    {
    intakeSetpoint = (-INTAKE_SPEED);
    }
  else if ((master.get_digital(DIGITAL_L1)) || (master.get_digital(DIGITAL_A)) || (master.get_digital(DIGITAL_X)))
    {
    //Forwards
    intakeSetpoint = (INTAKE_SPEED);
    }
  else
    {
    intakeSetpoint = (0);
    }
  }

void intakeSpin(void*)
  {
  //DIGITAL_L2 is the button assigned to the intakes
  while (true)
    {
    if (intakeOn)
      {
      if (STOP)
        {
        intakeSetpoint = 0;
        }
      leftIntake.move_voltage(intakeSetpoint);
      rightIntake.move_voltage(intakeSetpoint);
      s__t(0, t__s(leftIntake.get_actual_velocity()));
      }
    pros::delay(DELAY_INTERVAL);
    }
  }

const void controllerIndexerSpin(void)
  {
  if (master.get_digital(DIGITAL_Y) || master.get_digital(DIGITAL_R2))
    {
    indexerSetpoint = (INDEXER_SPEED);
    }
  else if (master.get_digital(DIGITAL_A) || master.get_digital(DIGITAL_L1))
    {
    indexerSetpoint = (-INDEXER_SPEED);
    }
  else
    {
    indexerSetpoint = (0);
    }
  s__t(0, "indexer time: " + t__s(pros::millis()) + " : " + t__s(master.get_digital(DIGITAL_R2)));
  }

PID indexerPID = PID(1, 0, 0);

void indexerSpin(void*)
  {
  //DIGITAL_R2 is the button assigned to the indexer
  while (true)
    {
    if (indexerOn)
      {
      if (STOP)
        {
        indexerSetpoint = 0;
        }


      if (indexerSetpoint == 0)
        {
        double indexerVoltage = indexerPID.Update(0, indexer.get_actual_velocity());
        indexer.move(indexerVoltage);
        }
      else
        {
        indexer.move_voltage(indexerSetpoint);
        }
      }
    pros::delay(DELAY_INTERVAL);
    }
  }

bool setDriveSafe(double leftVelocity, double rightVelocity) {
  //Scale velocity (0-200) to a voltage value (0-12000 mV)
  // int leftVoltage = (int) fmax(fmin(leftVelocity * 60, 12000), -12000);
  // int rightVoltage = (int) fmax(fmin(rightVelocity * 60, 12000), -12000);
  activeDriveMode = ManualMode;

  int leftVoltage = leftVelocity * 60;
  if (abs(leftVoltage) > 8000) {
    leftVoltage = 8000 * sgn(leftVoltage);
    }
  else if (abs(leftVoltage) < 3500) {
    leftVoltage = 3500 * sgn(leftVoltage);
    }

  int rightVoltage = rightVelocity * 60;
  if (abs(rightVoltage) > 8000) {
    rightVoltage = 8000 * sgn(rightVoltage);
    }
  else if (abs(rightVoltage) < 3500) {
    rightVoltage = 3500 * sgn(rightVoltage);
    }
  s__t(6, t__s(leftVoltage) + " " + t__s(rightVoltage));

  //Take the mutex to avoid writing to the same location twice
  // if (driveControl.take(0)) { //0 indicates the max number of milliseconds to wait before moving on
  if (true) {
    leftFront->move_voltage(leftVoltage);
    leftBack->move_voltage(leftVoltage);
    rightFront->move_voltage(rightVoltage);
    rightBack->move_voltage(rightVoltage);
    //Release the mutex
    // driveControl.give();
    return true; //success
    }
  return false; //Could not set value
  }

//Similar functions to setDriveSafe, for different parts of the robot
bool setIntakesSafe(double velocity) {
  int voltage = (int)(velocity * 60);
  if (intakeControl.take(0)) {
    intakeSetpoint = (voltage);
    intakeControl.give();
    return true;
    }
  return false;
  }

bool setIndexerSafe(double velocity) {
  int voltage = (int)(velocity * 60);
  if (indexerControl.take(0)) {
    indexerSetpoint = voltage;
    indexerControl.give();
    return true;
    }
  return false;
  }

bool setShooterSafe(double velocity) {
  int voltage = (int)(velocity * 60);
  if (shooterControl.take(0)) {
    shooterSetpoint = (voltage);
    shooterControl.give();
    return true;
    }
  return false;
  }
