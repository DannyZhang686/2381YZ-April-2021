#include "main.h"
#include "opcontrol.h"
#include "motors.h"
#include "utilities.h"
#include "control/motor_controller.hpp"
#include "control/pid.hpp"

#define DEAD_ZONE 20 //The joystick deadzone in percent of half the joystick range
#define MAX_DELTA_SPEED 5000 //The maximum permitted change in target speed
#define DELAY_INTERVAL 20
using namespace std;
using namespace pros;
// PID ARRAY
array<double, 3> left_back_pid_values = {0.6, 0, 0};
array<double, 3> right_back_pid_values = {0.6, 0, 0};
array<double, 3> left_front_pid_values = {0.6, 0, 0};
array<double, 3> right_front_pid_values = {0.6, 0, 0};

array<double, 3> master_drive_pid_values = {0, 0.01, 0};

std::array<double,4> _pid_inputs = {0,0,0,0};
auto _master_pid = new PID((master_drive_pid_values)[0], (master_drive_pid_values)[1], (master_drive_pid_values)[2]);

const void InitMotorControllers(DrivePidConfig config)
{
  _left_front_motor_controller = new Motor_Controller(config[left_front][0], config[left_front][1], config[left_front][2], leftFront);
  _left_back_motor_controller = new Motor_Controller(config[left_back][0], config[left_back][1], config[left_back][2], leftBack);
  _right_front_motor_controller = new Motor_Controller(config[right_front][0], config[right_front][1], config[right_front][2], rightFront);
  _right_back_motor_controller = new Motor_Controller(config[right_back][0], config[right_back][1], config[right_back][2], rightBack);

};

// SET DRIVE VARIABLES DEFINED HERE
  double _left_front_setpoint;
  double _left_back_setpoint;
  double _right_front_setpoint;
  double _right_back_setpoint;
  
  double _master_offset = 1;

  double lboffset = 1, rboffset = 1, rfoffset = 1, lfoffset = 1;
  double lbDistance = 1, rbDistance = 1, rfDistance = 1, lfDistance = 1, masterDistance = 1;

  double _left_back_motor_value, _left_front_motor_value, _right_back_motor_value, _right_front_motor_value;

  double _motor_value_average;
  double _master_setpoint = 0;  
  double _previous_setpoint;
  double _master_error_average = 0;


// RATIO CALC FUNCTION USED FOR PID VALUES IN THE SET DRIVE FUNCTION
double ratioCalc(double masterDis, double masterOS, double specDis, double specOS)
{
  if (masterOS == 0 || specDis == 0 || masterDis == 0 || specOS == 0)
  {
    return 1;
  }
  return pow((masterDis * specOS / (masterOS * specDis)), 6);
}


// SET DRIVE FUNCTION CALLED BY MOVE_MOTOR
void Set_Drive(double left_x, double left_y, double right_x, double right_y)
{
  s__t(5, "REEE");
  _motor_value_average = (abs(_left_back_motor_value) + abs(_left_front_motor_value) + abs(_right_back_motor_value) + abs(_right_front_motor_value)) / 4;
  //motor_value_average is what the actual motors are currently set at

  if (_master_setpoint >= 0) //setpoint is what we as controllers want the code to actually output
  {
    _master_error_average = _motor_value_average - _master_setpoint; //master error is used in the pid values to tune the motor values
  }
  else
  {
    _master_error_average = _master_setpoint - _motor_value_average;
  }
  _left_back_setpoint = (left_y - left_x + pow((std::abs(right_x) / 127), 0.8) * (right_x));
  _left_front_setpoint = (left_y + left_x + pow((std::abs(right_x) / 127), 0.8) * (right_x));
  _right_back_setpoint = (left_y + left_x - pow((std::abs(right_x) / 127), 0.8) * (right_x));
  _right_front_setpoint = (left_y - left_x - pow((std::abs(right_x) / 127), 0.8) * (right_x));

  _master_setpoint = (abs(_left_back_setpoint) + abs(_left_front_setpoint) + abs(_right_back_setpoint) + abs(_right_front_setpoint)) / 4;
  _master_offset += (_master_setpoint);

  lfoffset += (abs(_left_front_setpoint));
  rboffset += (abs(_right_back_setpoint));
  lboffset += (abs(_left_back_setpoint));
  rfoffset += (abs(_right_front_setpoint));

  rfDistance += abs(_right_front_motor_controller->Get_Speed()) / DELAY_INTERVAL;
  lfDistance += abs(_left_front_motor_controller->Get_Speed()) / DELAY_INTERVAL;
  rbDistance += abs(_right_back_motor_controller->Get_Speed()) / DELAY_INTERVAL;
  lbDistance += abs(_left_back_motor_controller->Get_Speed()) / DELAY_INTERVAL;

  masterDistance += (abs(_right_front_motor_controller->Get_Speed()) + abs(_left_back_motor_controller->Get_Speed()) + abs(_right_back_motor_controller->Get_Speed()) + abs(_left_front_motor_controller->Get_Speed())) / (4 * DELAY_INTERVAL);

  double tuning_coefficient = _master_pid->Update(0, _master_error_average);
  if (tuning_coefficient < 0)
  {
    _master_pid->ResetError();
    tuning_coefficient = 1;
  }

  _pid_inputs[left_back] = _left_back_setpoint * ratioCalc(masterDistance, _master_offset, lbDistance, lboffset) * tuning_coefficient;
  _pid_inputs[left_front] = _left_front_setpoint * ratioCalc(masterDistance, _master_offset, lfDistance, lfoffset) * tuning_coefficient;
  _pid_inputs[right_back] = _right_back_setpoint * ratioCalc(masterDistance, _master_offset, rbDistance, rboffset) * tuning_coefficient;
  _pid_inputs[right_front] = _right_front_setpoint * ratioCalc(masterDistance, _master_offset, rfDistance, rfoffset) * tuning_coefficient;

  if (master.get_digital(DIGITAL_A))
  {
    lcd::set_text(4, to_string((int)_pid_inputs[left_back]) + ":" + to_string((int)_pid_inputs[left_front]) + ":" + to_string((int)_pid_inputs[right_back]) + ":" + to_string((int)_pid_inputs[right_front]));
    lcd::set_text(5, to_string((int)_left_back_setpoint) + ":" + to_string((int)_left_front_setpoint) + ":" + to_string((int)_right_back_setpoint) + ":" + to_string((int)_right_front_setpoint));
    lcd::set_text(6, to_string((int)_left_back_motor_value) + ":" + to_string((int)_left_front_motor_value) + ":" + to_string((int)_right_back_motor_value) + ":" + to_string((int)_right_front_motor_value));
  }
}


void Move_Motor(void*)
{

  while(true) {
    Set_Drive(0, master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X), master.get_analog(ANALOG_RIGHT_Y));
    _left_back_motor_value = _left_back_motor_controller->Set_Speed(_pid_inputs[left_back]);
    _left_front_motor_value = _left_front_motor_controller->Set_Speed(_pid_inputs[left_front]);
    _right_back_motor_value = _right_back_motor_controller->Set_Speed(_pid_inputs[right_back]);
    _right_front_motor_value = _right_front_motor_controller->Set_Speed(_pid_inputs[right_front]);

    pros::delay(20);
  }

}
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

    if ((fabs(left) > DEAD_ZONE) || (fabs(right) > DEAD_ZONE)) {
      //Each of left and right are now mapped to a value between
      //0 and 12000 using a custom exponential curve to ensure precise
      //micro movement (especially turns) while maintaining high max speed.
      //See http://bit.ly/voltagecurve for a graph of input vs. output.

      left = sgn(left) * (floor(52.5 * exp(fabs(left) / 19.0)) + 1862);
      right = sgn(right) * (floor(52.5 * exp(fabs(right) / 19.0)) + 1862);
    }
    else {
      //Move at voltage 0 if there's very little or no input
      left = right = 0;
    }

    //Limit the maximum change in motor speed to prevent tipping and chain snapping
    actualLeftSpeed = (leftFront->get_actual_velocity() + leftBack->get_actual_velocity()) * 30; //Multiplied by 0.5 * 60 for average and conversion from rpm (0-200) to equivalent mV (0-12000)
    actualRightSpeed = (rightFront->get_actual_velocity() + rightBack->get_actual_velocity()) * 30;
    if (fabs(left - actualLeftSpeed) > MAX_DELTA_SPEED) {
      //Outside the allowable range; set speed as appropriate
      if (left > actualLeftSpeed) {
        left = actualLeftSpeed + MAX_DELTA_SPEED;
      }
      else {
        left = actualLeftSpeed - MAX_DELTA_SPEED;
      }
    }
    if (fabs(right - actualRightSpeed) > MAX_DELTA_SPEED) {
      if (right > actualRightSpeed) {
        right = actualRightSpeed + MAX_DELTA_SPEED;
      }
      else {
        right = actualRightSpeed - MAX_DELTA_SPEED;
      }
    }
    leftFront->move_voltage((int) left);
    leftBack->move_voltage((int) left);
    rightFront->move_voltage((int) right);
    rightBack->move_voltage((int) right);
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
    if ((master.get_digital(DIGITAL_R1)) || (master.get_digital(DIGITAL_A))) {
      shooter.move_voltage(-SHOOTER_SPEED);
    }
    else if (master.get_digital(DIGITAL_X)) {
      shooter.move_voltage(SHOOTER_SPEED);
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
