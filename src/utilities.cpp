#include "utilities.h"

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
