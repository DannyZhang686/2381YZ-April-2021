#include "main.h"
#include "motors.h"
#include "autonomous.h"
#include "utilities.h"

//Functions for miscellaneous use in autonomous

void setDriveSafe(double leftVelocity, double rightVelocity) {
  //Scale velocity (0-200) to a voltage value (0-12000 mV)
  int leftVoltage = (int) (leftVelocity * 600);
  int rightVoltage = (int) (rightVelocity * 600);
  //Take the mutex to make sure this is the only code setting voltages
  //If unsuccessful, drive values can be set at the next PID refresh
  if (driveCommand.take(0)) { //0 indicates the max number of milliseconds to wait before moving on
    leftFront.move_voltage(leftVoltage);
    leftBack.move_voltage(leftVoltage);
    rightFront.move_voltage(rightVoltage);
    rightBack.move_voltage(rightVoltage);
    //Release the mutex
    driveCommand.give();
  }
}

double findDistance (Point a, Point b) {
  //Simple function to find distance between two points using a standard distance formula
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

//Functions for convenience
int sgn(int i) {
  if (i > 0) return 1;
  else if (i == 0) return 0;
  else return -1;
}

int sgn(float f) {
  if (f > 0) return 1;
  else if (f == 0) return 0;
  else return -1;
}

int sgn(double d) {
  if (d > 0) return 1;
  else if (d == 0) return 0;
  else return -1;
}

int sgn(bool b) {
  if (b) return 1;
  else if (!b) return -1;
}

double encToInches(int encoderValue) {
  //Converts an encoder value (in degrees) to a distance traveled (in inches)
  return encoderValue * PI * TRACKING_WHEEL_DIAMETER / 360.0;
}
