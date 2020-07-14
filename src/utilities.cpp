#include "main.h"
#include "utilities.h"

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
