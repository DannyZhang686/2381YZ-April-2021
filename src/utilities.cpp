#include "main.h"
#include "motors.h"
#include "autonomous.h"
#include "utilities.h"

//Functions for miscellaneous use in autonomous

void setDriveSafe(double leftVelocity, double rightVelocity) {
  //Scale velocity (0-200) to a voltage value (0-12000 mV)
  int leftVoltage = (int) (leftVelocity * 60);
  int rightVoltage = (int) (rightVelocity * 60);
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

double findDistance(Point a, Point b) {
  //Function to find distance between two Points using a standard distance formula
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double findAngle(Point a, Point b) {
  //Function to find angle between two Points starting from Point a
  //This function always returns a value between 0 and 2π
  double diffX = b.x - a.x, diffY = b.y - a.y;
  int quadrant = 0; //The quadrant b is in if a represents (0, 0) on an xy-plane
  //Note that conventional quadrants are used; however, the return angle
  //is measured clockwise from the positive y-axis for ease of use
  if ((diffX == 0) || (diffY == 0)) { //If b is at a right angle to, or directly in front of/behind a
    if (diffX == 0) { //Point b is on y-axis
      if (diffY < 0) return PI;
      else return 0;
    }
    else { //diffX == 0, Point b is on x-axis
      if (diffX > 0) return PI / 2;
      else return 3 * PI / 2;
    }
  }

  if (diffX > 0) {
    if (diffY > 0) quadrant = 1;
    else quadrant = 4;
  }
  else { //diffX < 0
    if (diffY > 0) quadrant = 2;
    else quadrant = 3;
  }

  diffX = fabs(diffX); //For ease of use in the following section
  diffY = fabs(diffY);

  //Calculate the correct return values, noting the 1-4-3-2 order of the quadrants
  if (quadrant == 1) return atan(diffX / diffY);
  else if (quadrant == 4) return PI - atan(diffX / diffY);
  else if (quadrant == 3) return PI + atan(diffX / diffY);
  else return 2 * PI - atan(diffX / diffY); //quadrant == 2
}

double smallestAngle(double current, double target) {
  //Function to find the smallest (optimal) angle difference between two angular positions
  //This function should always return a value between -π and π
  double diff = target - current;
  //Note that the maximum difference between two angles in the range of 0-2π is 2π,
  //and the minimum is -2π; thus, simply adding or subtracting 2π from one of the
  //angles (which maintains rotation) is sufficient to guarantee the smallest answer
  if ((diff <= PI) && (diff >= -PI)) return diff;
  diff += 2 * PI;
  if ((diff <= PI) && (diff >= -PI)) return diff;
  diff -= 4 * PI; //-2π from two lines ago and -2π again
  return diff; //Guaranteed to be correct
}

double angleToInches(double angle) {
  //Function to convert an angular measurement to a value (in inches) to be travelled by the drivebase
  //This should be unique to each robot and depend on the positioning of the wheels
  //Note that there is a simple linear correlation between angle and return value
  double coefficient = 0; //TBD with testing
  return angle * coefficient;
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
