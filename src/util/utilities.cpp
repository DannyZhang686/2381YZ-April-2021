#include "main.h"
#include "utilities.h"

//Functions for convenience

const double powCalc(double x, double power)
    {
    return pow(abs(x), power) * getSignOf(x);
    }


const double getSignOf(const double signedVar)
    {
    if (signedVar >= 0)
        {
        return 1;
        }
    return -1;
    }

const double minMaxMod(const double k, const double max, const double min)
    {
    if (k > max) return max;
    if (k < min) return min;
    return k;
    }

const double minMaxMod(const double k, const double max)
    {
    return minMaxMod(k, abs(max), -abs(max));
    }

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
  else return -1;
}

double encToInches(int encoderValue) {
  //Converts an encoder value (in degrees) to a distance traveled (in inches)
  return encoderValue * PI * TRACKING_WHEEL_DIAMETER / 360.0;
}

double rotatePi(double angle) {
  //Rotates an angle by π radians, returning a value between 0 and 2π
  angle = fmod(angle, 2 * PI);
  if (sgn(angle) == -1) {
    angle += 2 * PI;
  }
  //Angle is now between 0 and 2π
  angle += PI; //Angle is now between π and 3π
  if (angle >= 2 * PI) { //Ensure that the return value is between 0 and 2π
    angle -= 2 * PI;
  }
  return angle;
}

double degToRad (double deg) {
  //Converts an angle from a degree value to a radian value
  return deg * PI / 180.0;
}

double radToDeg (double rad) {
  //Converts an angle from a radian value to a degree value
  return rad * 180.0 / PI;
}
