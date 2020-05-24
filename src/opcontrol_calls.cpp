#include "main.h"
#include "opcontrol.h"
#include "motors.h"

void splitArcade(void* driveValues) {
  std::pair<int, int> stuff = *static_cast<std::pair<int, int> *> (driveValues);
  int left = stuff.first + stuff.second;
  int right = stuff.first - stuff.second;
  leftFront.move_velocity(left);
  leftBack.move_velocity(left);
  rightFront.move_velocity(right);
  rightBack.move_velocity(right);
}

void flywheel(void*) {
  if (master.get_digital(DIGITAL_R1)) {
    //flywheel goes wheee
  }
}
