#include "main.h"
#include "autonomous.h"
#include "motors.h"
#include "utilities.h"

//Extern variable declarations

//Robot position and encoder values structs
Position robotPos;
EncoderVal currentVal, lastVal;
DeltaVal deltaVal;

//Mutexes
pros::Mutex driveCommand;

void trackPosition(void*) {
  //This runs in a Task and thus requires an infinite loop.
  while (true) {
    //Update the current values
    currentVal.left = leftTracking.get_value();
    currentVal.right = rightTracking.get_value();
    currentVal.back = backTracking.get_value();

    //Calculate the change since previous time
    //Note that deltaVal is in inches and not encoder units, hence the conversion.
    deltaVal.left = encToInches(currentVal.left - lastVal.left);
    deltaVal.right = encToInches(currentVal.right - lastVal.right);
    deltaVal.back = encToInches(currentVal.back - lastVal.back);

  	//Update the previous values (for use the next time)
    lastVal.left = currentVal.left;
    lastVal.right = currentVal.right;
  	lastVal.back = currentVal.back;

    //Consider the tracking center at the last calculation and the current
    //calculation: due to the frequency at which they are recalculated, they
    //can be approximated as points on an arc traveling around a circle

    double rRadius; //The radius of the circle calculated to the right tracking wheel
    double bRadius; //rRadius calculated for the back tracking wheel (used as an adjustment for when the robot turns or is pushed sideways)
    double dist; //The distance travelled by the tracking center
    double dist2; //dist calculated using the back tracking wheel
    double angleTravelled = (deltaVal.left - deltaVal.right) / (L_TO_MID + R_TO_MID); //Radian angle travelled (clockwise is positive)
    double sinAngle = sin(angleTravelled); //The sine of the angle (used to avoid multiple redundant calculations)

  	if (angleTravelled != 0) {
  		rRadius = deltaVal.right / angleTravelled; //Calculate the radius
  		dist = (rRadius + R_TO_MID) * sinAngle; //Calculate the distance (from the center of the robot) using simple trigonometry
  		bRadius = deltaVal.back / angleTravelled; //Repeat the previous lines using the back tracking wheel
  		dist2 = (bRadius + B_TO_MID) * sinAngle;
  	}
    else {
      //Robot went straight or didn't move; note that this happens when deltaVal.right == deltaVal.left
      //Values for distance travelled can be set directly to encoder values, as there is no arc
  		dist = deltaVal.right;
  		dist2 = deltaVal.back;
  	}

  	robotPos.angle += angleTravelled; //Calculate the updated angle value
    double sinTheta = sin(robotPos.angle); //Precalculate the sine and cosine of the final angle to avoid redundancy
    double cosTheta = cos(robotPos.angle);

  	//Calculate updated absolute position values
    //Note that these specific signs and ratios are used because:
    // - dist is calculated using an angle clockwise from the positive y-axis (hence sin is x and cos is y); and
    // - dist2 is calculated using an angle clockwise from the positive x-axis (hence cos is x and -sin is y)
    robotPos.x += dist * sinTheta + dist2 * cosTheta;
    robotPos.y += dist * cosTheta - dist2 * sinTheta;

    //Print the tracking values to the brain screen for debugging
    // s__t(1, "");
    pros::delay(10);
  }
}

void moveShort(double targetX, double targetY) {
  //Moves with a full PID
  Point current; //Current coordinates as per tracking
  Point target(targetX, targetY); //Target coordinates

  //Movement loop
  do {
    current.setValues(robotPos.x, robotPos.y);
  } while (false);
}

void moveLong(double targetX, double targetY, double moveShortDistance) {
  //Moves with position tracking and ends with PID
  Point current; //Current coordinates as per tracking
  Point target(targetX, targetY); //Target coordinates

  //Movement loop
  do {
    current.setValues(robotPos.x, robotPos.y);
  } while (findDistance(current, target) > moveShortDistance);

  //Break the loop when sufficiently close to target, and use moveShort for the rest.
  moveShort(target.x, target.y);
}

void turnToFace(double angle) {
  //turns with full PID
}

//Autonomous routines

void movementOne(void*) {}

void snailOne(void*) {}

void movementTwo(void*) {}

void snailTwo(void*) {}
