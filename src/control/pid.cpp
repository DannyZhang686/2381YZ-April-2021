#include "main.h"
#include "pid.h"

//Core functionality code for both PID and PD controller

double Pid::getOutput(double currentValue, double setpoint) {
  double error = setpoint - currentValue; //Distance from the target
  double p = kP * error; //P component of return velocity
  errorSum += error; //Add to the integral error
  double i = kI * errorSum; //I component of return velocity == kI * integral
  double d = kD * (error - prevError); //D component of return velocity == kD * derivative error
  prevError = error; //Update the previous error
  return p + i + d;
}

double Pd::getOutput(double currentValue, double setpoint) {
  double error = setpoint - currentValue; //Distance from the target
  double p = kP * error; //P component of return velocity
  double d = kD * (error - prevError); //D component of return velocity == kD * derivative error
  prevError = error; //Update the previous error
  return p + d;
}
