#include "main.h"
#include "autonomous.h"
#include "motors.h"
#include "utilities.h"
#include "pid.h"
#include "globals.hpp"

//Robot position and encoder values structs
Position robotPos;
EncoderVal currentVal, lastVal;
DeltaVal deltaVal;

//PD controllers for the drive, including tuned kP and kD values
//There are two controllers per side to account for both linear and angular error
//They are tuned to the same kP and kD values, but will concurrently have different error values
double leftkP = 1, leftkD = 3;
double rightkP = 1, rightkD = 3;
// double leftkP = 15.6, leftkD = 0.5335; //speculative tuned values
// double rightkP = 15.6, rightkD = 0.5335;

//Mutexes
pros::Mutex pdGetOutput;
double gyroAngle;

//Tracking algorithms
void updateTracking(void)
{
  //This runs in a Task and thus requires an infinite loop.

  if (inertial->IsCalibrating())
  {
    s__t(1, "IMU CALIBRATING");
    return;
  }


  //Update the current values
  inertial->Update_Gyro();
  currentVal.left = leftTracking->get_value();
  // currentVal.right = rightTracking.get_value();
  currentVal.back = backTracking->get_value();
  // currentVal.angle = inertial->Get_Angle();
  if (currentVal.angle > 2 * PI)
    currentVal.angle -= 2 * PI;
  if (currentVal.angle < 0)
    currentVal.angle += 2 * PI;

  //Find the change since the previous calculation
  //Note that deltaVal is in inches and not encoder units, hence the conversion.
  deltaVal.left = encToInches(currentVal.left - lastVal.left);
  // deltaVal.right = encToInches(currentVal.right - lastVal.right);
  deltaVal.back = encToInches(currentVal.back - lastVal.back);
  deltaVal.angle = currentVal.angle - lastVal.angle;
  // deltaVal.angle = smallestAngle(currentVal.angle, lastVal.angle);

  //Update the previous values (for use the next time)
  lastVal.left = currentVal.left;
  // lastVal.right = currentVal.right;
  lastVal.back = currentVal.back;
  lastVal.angle = currentVal.angle;

  //Consider the tracking center at the last calculation and the current
  //calculation: due to the frequency at which they are recalculated, they
  //can be approximated as points on an arc traveling around a circle

  double lRadius;                        //The radius of the circle calculated to the left tracking wheel
  double bRadius;                        //rRadius calculated for the back tracking wheel (used as an adjustment for when the robot turns or is pushed sideways)
  double dist;                           //The distance travelled by the tracking center
  double dist2;                          //dist calculated using the back tracking wheel
  double sinAngle = sin(deltaVal.angle); //The sine of the angle travelled (used to avoid multiple redundant calculations)

  if (fabs(deltaVal.angle) > 0)
  {
    lRadius = deltaVal.left / deltaVal.angle; //Calculate the radius
    // dist = (rRadius + R_TO_MID) * deltaVal.angle; //Calculate the distance (from the center of the robot) using simple trigonometry
    dist = (lRadius - L_TO_MID) * sinAngle;   //Calculate the distance (from the center of the robot) using simple trigonometry
    bRadius = deltaVal.back / deltaVal.angle; //Repeat the previous lines using the back tracking wheel (for horizontal error)
    dist2 = (bRadius + B_TO_MID) * sinAngle;
  }
  else
  {
    //Robot went straight or didn't move; note that this happens when deltaVal.angle == 0
    //Values for distance travelled can be set directly to encoder values, as there is no arc
    dist = deltaVal.left;
    dist2 = deltaVal.back;
    deltaVal.angle = 0;
  }

  robotPos.angle = currentVal.angle;     //Update the angle value
  double sinTheta = sin(robotPos.angle); //Calculate the sine and cosine of the final angle to avoid redundancy
  double cosTheta = cos(robotPos.angle);

  //Calculate updated absolute position values
  //Note that these specific signs and ratios are used because:
  // - dist is calculated using an angle clockwise from the positive y-axis (hence sin is x and cos is y); and
  // - dist2 is calculated using an angle clockwise from the positive x-axis (hence cos is x and -sin is y)
  robotPos.x += dist * sinTheta + dist2 * cosTheta;
  robotPos.y += dist * cosTheta - dist2 * sinTheta;

  //Print the tracking values to the brain screen for debugging
  s__t(0, t__s(currentVal.left) + " " + t__s(currentVal.back));
  s__t(1, t__s(robotPos.x) + " " + t__s(robotPos.y) + " " + t__s(robotPos.angle));
}

void trackPosition(void *)
{
  while (true)
  {
    position_tracker->Track_Position();
    pros::delay(DELAY_INTERVAL);
  }
}

//Movement algorithms
void moveShort(double targetX, double targetY, double maxError, bool forceForward)
{
  //Moves and turns with a full PD
  //Note: this is meant for small turns, big turns are handled by turnToFace
  Pd leftStraight(leftkP, leftkD);
  Pd leftTurn(leftkP, leftkD);
  Pd rightStraight(rightkP, rightkD);
  Pd rightTurn(rightkP, rightkD);
  OPoint current;                                  //Current coordinates as per tracking
  OPoint target(targetX, targetY);                 //Target coordinates
  double distance, targetAngle;                   //Distance/Angle between current and target
  double travellingAngle;                         //The angle (-π to π, though -π/4 to π/4 works much better) to travel to face the target
  double tAngleInches;                            //travellingAngle converted to a value in inches (for PD purposes)
  double lastLeftOutput = 0, lastRightOutput = 0; //The wheel output to be used in case the mutex is unavailable
  bool goBackward = false;                        //Whether to move backward instead of forward to the target

  if (!forceForward)
  { //Consider moving backward to get to the target
    targetAngle = findAngle(current, target);
    travellingAngle = smallestAngle(robotPos.angle, targetAngle);
    if (fabs(travellingAngle) >= PI / 2)
    { //Moving backward is better (smaller turn)
      goBackward = true;
    }
  }

  //Movement loop
  do
  {
    //Update distance and angle variables
    current.setValues(robotPos.x, robotPos.y);
    distance = findDistance(current, target);
    targetAngle = findAngle(current, target);
    travellingAngle = smallestAngle(goBackward ? rotatePi(robotPos.angle) : robotPos.angle, targetAngle);
    tAngleInches = angleToInches(travellingAngle);

    if (pdGetOutput.take(0))
    {                                         //Access to PID
      double leftOutput = 0, rightOutput = 0; //Power output (0-200) for each side of the robot
      //Calculate new drive values, adding together linear and angular values for a final number
      leftOutput += leftStraight.getOutput(0, distance); //Call to PID to find velocity
      rightOutput += rightStraight.getOutput(0, distance);
      leftOutput += 7 * leftTurn.getOutput(0, tAngleInches); //Setpoint distance value for PD
      rightOutput -= 7 * rightTurn.getOutput(0, tAngleInches);
      if (goBackward)
      {
        leftOutput = -leftOutput;
        rightOutput = -rightOutput;
      }
      lastLeftOutput = leftOutput; //Update the latest available values
      lastRightOutput = rightOutput;
      s__t(2, "outputs" +  t__s(leftOutput) + " " + t__s(rightOutput));
      s__t(3, "dist, pos" + t__s(distance) + " " + t__s(current.x) + " " + t__s(current.y));
      setDriveSafe(leftOutput, rightOutput);
      pdGetOutput.give();
    }
    else
    {                                                //Something else is using the PID
      setDriveSafe(lastLeftOutput, lastRightOutput); //Use previous values
    }
    pros::delay(20);
  } while (distance > maxError);
  //Set motors to a small opposite value for a short time to stop on the spot
  setDriveSafe(-lastLeftOutput * 6, -lastRightOutput * 6);
  pros::delay(100);
  setDriveSafe(0, 0);
}

//This function might not be necessary if moveShort works well enough
void moveLong(double targetX, double targetY, double moveShortDistance, double maxError)
{
  //Moves with position tracking and ends with PD
  OPoint current;                  //Current coordinates as per tracking
  OPoint target(targetX, targetY); //Target coordinates

  //Movement loop
  do
  {
    current.setValues(robotPos.x, robotPos.y);
  } while (findDistance(current, target) > moveShortDistance);

  //Break the loop when sufficiently close to target, and use moveShort for the rest.
  moveShort(target.x, target.y, maxError, false);
}

void movePurePursuit(double targetX, double targetY, double maxError, double lookDist, bool goForward)
{
  Pd leftStraight(leftkP, leftkD);
  Pd leftTurn(leftkP, leftkD);
  Pd rightStraight(rightkP, rightkD);
  Pd rightTurn(rightkP, rightkD);

  OPoint current(robotPos.x, robotPos.y); //Current coordinates as per tracking
  OPoint target(targetX, targetY);        //Target coordinates
  OPoint goalPoint;                       //Point the robot will chase
  Line movingLine(current, target);      //Line between starting and target coordinates
  double distance, targetAngle;          //Distance/Angle between current and target
  double travellingAngle;                //The angle (-π to π, though -π/4 to π/4 works much better) to travel to face the target
  double tAngleInches;                   //travellingAngle converted to a value in inches (for PD purposes)
  int time = 0;
  bool setTime = false;

  OPoint starting(robotPos.x, robotPos.y);
  distance = findDistance(starting, target);

  do
  {
    //Update current robot position
    current.setValues(robotPos.x, robotPos.y);
    goalPoint = movingLine.findGoalPoint(current, lookDist);

    // distance = findDistance(current, goalPoint);
    targetAngle = findAngle(current, goalPoint);
    if (goForward)
    {
      travellingAngle = smallestAngle(robotPos.angle, targetAngle);
    }
    else
    {
      travellingAngle = smallestAngle(rotatePi(robotPos.angle), targetAngle);
    }
    tAngleInches = angleToInches(travellingAngle);

    double leftOutput = 0, rightOutput = 0; //Power output (0-200) for each side of the robot
    //Calculate new drive values, adding together linear and angular values for a final number
    // leftOutput += leftStraight.getOutput(0, 15*distance); //Call to PID to find velocity
    // rightOutput += rightStraight.getOutput(0, 15*distance);
    leftOutput += leftStraight.getOutput(15 * findDistance(starting, current), 15 * distance); //Call to PID to find velocity
    rightOutput += rightStraight.getOutput(15 * findDistance(starting, current), 15 * distance);
    if (time == 0)
    {
      leftOutput += 3 * leftTurn.getOutput(0, 15 * tAngleInches); //Setpoint distance value for PD
      rightOutput -= 3 * rightTurn.getOutput(0, 15 * tAngleInches);
    }
    if (!goForward)
    {
      leftOutput = -leftOutput;
      rightOutput = -rightOutput;
    }
    s__t(2, t__s(leftOutput) + " " + t__s(rightOutput));
    s__t(3, t__s(distance) + " " + t__s(current.x) + " " + t__s(current.y));
    setDriveSafe(leftOutput, rightOutput);
    pros::delay(20);
    if ((!setTime) && (findDistance(current, target) < maxError))
    {
      time = pros::millis();
      setTime = true;
    }
    if ((time != 0) && (pros::millis() - time > 300))
    {
      break;
    }
  } while (true);
  setDriveSafe(0, 0);
}

void approachGoal(double velocity, double time)
{
  //Approaching a goal; PID/tracking can't be used because tiny
  //errors will destroy the auto (run against goal forever)
  setDriveSafe(velocity, velocity);
  pros::delay(time);
  setDriveSafe(0, 0);
}

void backAway(double velocity, double time)
{
  //Same as approaching the goal, but with negative velocity
  approachGoal(-velocity, time);
}

Pd leftTurn(leftkP, leftkD);
Pd rightTurn(rightkP, rightkD);

void turnToFace(double targetAngle, double maxError)
{
  //Complete turn (no simultaneous drive) with full PD

  double travellingAngle; //The angle (-π to π) to travel to face the final angle
  double tAngleInches;    //travellingAngle converted to a value in inches (for PD purposes)
  int time = 0;
  bool setTime = false;

  do
  {
    travellingAngle = smallestAngle(robotPos.angle, targetAngle);
    tAngleInches = angleToInches(travellingAngle);

    double leftOutput = 0, rightOutput = 0;                //Power output (0-200) for each side of the robot
    leftOutput = leftTurn.getOutput(0, 25 * tAngleInches); //Setpoint distance value for PD
    rightOutput = -rightTurn.getOutput(0, 25 * tAngleInches);
    setDriveSafe(leftOutput, rightOutput);
    s__t(4, t__s(targetAngle) + " " + t__s(robotPos.angle) + " " + t__s(travellingAngle));
    pros::delay(20);
    if ((!setTime) && (fabs(travellingAngle) < maxError))
    {
      time = pros::millis();
      setTime = true;
    }
    else if ((time != 0) && (pros::millis() - time > 300))
    {
      break;
    }
  } while (true);
  setDriveSafe(0, 0);
}

void turnToPoint(double targetX, double targetY, double maxError)
{
  //Like turnToFace, but finding a point and continuously adjusting
  double targetAngle;     //The desired angle, which is constantly updated
  double travellingAngle; //The angle (-π to π) to travel to face the final angle
  double tAngleInches;    //travellingAngle converted to a value in inches (for PD purposes)
  int time = 0;
  bool setTime = false;

  do
  {
    targetAngle = findAngle(OPoint(robotPos.x, robotPos.y), OPoint(targetX, targetY));
    travellingAngle = smallestAngle(robotPos.angle, targetAngle);
    tAngleInches = angleToInches(travellingAngle);

    double leftOutput = 0, rightOutput = 0;                //Power output (0-200) for each side of the robot
    leftOutput = leftTurn.getOutput(0, 25 * tAngleInches); //Setpoint distance value for PD
    rightOutput = -rightTurn.getOutput(0, 25 * tAngleInches);
    setDriveSafe(leftOutput, rightOutput);
    s__t(4, t__s(targetAngle) + " " + t__s(robotPos.angle) + " " + t__s(travellingAngle));
    pros::delay(20);
    if ((!setTime) && (fabs(travellingAngle) < maxError))
    {
      time = pros::millis();
      setTime = true;
    }
    else if ((time != 0) && (pros::millis() - time > 250))
    {
      break;
    }
  } while (true);
  setDriveSafe(0, 0);
}

void turnAwayFromPoint(double targetX, double targetY, double maxError)
{
  //Like turnToPoint, but turning away
  double angle = findAngle(OPoint(robotPos.x, robotPos.y), OPoint(targetX, targetY));
  turnToFace(rotatePi(angle), maxError);
}

void turnToFaceOld(double targetAngle, double maxError)
{
  //Complete turn (no simultaneous drive) with full PD

  Pd leftTurn(leftkP, leftkD);
  Pd rightTurn(rightkP, rightkD);

  double travellingAngle;                         //The angle (-π to π) to travel to face the final angle
  double tAngleInches;                            //travellingAngle converted to a value in inches (for PD purposes)
  double lastLeftOutput = 0, lastRightOutput = 0; //The wheel output to be used in case the mutex is unavailable

  do
  {
    travellingAngle = smallestAngle(robotPos.angle, targetAngle);
    tAngleInches = angleToInches(travellingAngle);
    if (pdGetOutput.take(0))
    {                                                        //Access to PID
      double leftOutput = 0, rightOutput = 0;                //Power output (0-200) for each side of the robot
      leftOutput = leftTurn.getOutput(0, 15 * tAngleInches); //Setpoint distance value for PD
      rightOutput = -rightTurn.getOutput(0, 15 * tAngleInches);
      lastLeftOutput = leftOutput; //Update the latest available values
      lastRightOutput = rightOutput;
      setDriveSafe(leftOutput, rightOutput);
      s__t(2, t__s(leftOutput) + " " + t__s(rightOutput));
      s__t(3, t__s(tAngleInches) + " " + t__s(robotPos.angle) + " " + t__s(travellingAngle));
      pdGetOutput.give();
      pros::delay(20);
    }
    else
    {                                                //Something else is using the PID
      setDriveSafe(lastLeftOutput, lastRightOutput); //Use previous values
    }
  } while (fabs(travellingAngle) > maxError);
  //Set motors to an opposite value for a short time to stop on the spot
  setDriveSafe(-lastLeftOutput * 6, -lastRightOutput * 6);
  pros::delay(125);
  setDriveSafe(0, 0);
}

void turnToPointOld(double targetX, double targetY, double maxError)
{
  //Like turnToFace, but finding a point
  double angle = findAngle(OPoint(robotPos.x, robotPos.y), OPoint(targetX, targetY));
  // s__t(4, t__s(angle) + " from " + t__s(robotPos.x) + " " + t__s(robotPos.y));
  turnToFaceOld(angle, maxError);
}

//Ball manipulation (snail) functions

//Counting the number of balls shot from and intaken by the robot
//Starts at -0.5 due to initial addition of 0.5
double numBallsShot = -0.5, numBallsIntaken = -0.5;

void countBalls(void *)
{
  //Counts the number of balls shot out of the top of the robot
  //PROS API: "Line Trackers return a value between 0 and 4095, with 0 being the lightest reading and 4095 the darkest"
  //Thus larger return values indicate the presence of a ball (and vice versa)

  int minOutput = 2350;                     //Arbitrary minimum line sensor output where (it's assumed) there isn't a ball
  std::queue<int> tLastOutput, bLastOutput; //The last several outputs of each line sensor, stored in order
  int numOutputs = 5;                       //The number of outputs to be recorded in lastOutput (bigger number filters noise better but reacts to change slower)
  int tSumOutputs = 0, bSumOutputs = 0;     //The current sum of all elements in each lastOutput variable
  bool tIsBall = false, bIsBall = false;    //Whether or not the program believes there is a ball in front of each sensor
  //Initialization of lastOutput (there is no ball at the start)
  for (int i = 0; i < numOutputs; i++)
  {
    tLastOutput.push(0);
    bLastOutput.push(0);
  }
  while (true)
  {
    //Update variables
    tLastOutput.push(tLineSensor.get_value_calibrated());
    // bLastOutput.push(bLineSensor.get_value_calibrated());
    tSumOutputs += tLastOutput.back() - tLastOutput.front();
    // bSumOutputs += bLastOutput.back() - bLastOutput.front();
    tLastOutput.pop(); // bLastOutput.pop();
    if ((tSumOutputs / numOutputs > minOutput) == tIsBall) {
      //Nothing needs to be done, as the sensor output agrees with isBall
    }
    else
    {
      //There is a disagreement; switch tIsBall
      tIsBall = !tIsBall;
      numBallsShot += 0.5; //Adding 0.5 when the ball starts being shot and creating a total of 1 when the ball finishes shooting
    }
    //Same thing with the other line sensor
    // if ((bSumOutputs / numOutputs > minOutput) == bIsBall) {}
    // else {
    //   bIsBall = !bIsBall;
    //   numBallsIntaken += 0.5;
    // }
    // s__t(4, t__s(tLineSensor.get_value()) + " " + t__s(bLineSensor.get_value()) + " " + t__s(numBallsShot) + " " + t__s(numBallsIntaken));
    pros::delay(10);
  }
}

//The speed at which the motors will run


void intakeShoot(int numBallsIn, int numBallsOut)
{
  //Intake, index, and shoot the given number of balls (ex. cycling a tower)
  double initNumBallsShot = numBallsShot;       //Number of balls shot before this point
  double initNumBallsIntaken = numBallsIntaken; //Number of balls intaken before this point
  bool doneIntaking = false, doneShooting = false;

  int time = 0;
  bool setTime = false;

  //Spin everything
  setIntakesSafe(AUTO_INTAKE_VEL);
  setIndexerSafe(AUTO_INDEXER_VEL);
  setShooterSafe(AUTO_SHOOTER_VEL);
  while (true)
  {
    //Continue delaying until the numbers update
    if ((!setTime) && (initNumBallsShot + numBallsOut <= numBallsShot))
    {
      //Spin the shooter the other way instead, after a short delay
      time = pros::millis();
      setTime = true;
      s__t(3, "time set");
    }
    else if ((time != 0) && (pros::millis() - time > 100))
    {
      setShooterSafe(-AUTO_SHOOTER_VEL);
      doneShooting = true;
      s__t(4, "");
    }
    if ((numBallsIn == 0) || (initNumBallsIntaken + numBallsIn + 0.5 <= numBallsIntaken))
    {
      //Stop the intakes
      setIntakesSafe(0);
      doneIntaking = true;
    }
    if (doneShooting && doneIntaking)
    {
      break;
    }
    s__t(4, t__s(initNumBallsShot) + " " + t__s(numBallsOut) + " " + t__s(numBallsShot));
    s__t(5, t__s(initNumBallsIntaken) + " " + t__s(numBallsIn) + " " + t__s(numBallsIntaken));
    pros::delay(10);
  }
}

void intakeNoShoot(int time, double velocity)
{
  //Intake and index, running shooter backward to avoid shooting (ex. collecting balls)
  int endTime = pros::millis() + time;
  setIntakesSafe(velocity);
  setIndexerSafe(velocity);
  setShooterSafe(-velocity);
  while (pros::millis() < endTime)
  {
    pros::delay(10);
  }
}

void intakeNoShoot(double velocity)
{
  //Non-blocking version of above function, defaulting to a velocity of 200
  intakeNoShoot(0, velocity);
}

void discard(int time)
{
  //Run everything backward
  int endTime = pros::millis() + time;
  setIntakesSafe(-AUTO_INTAKE_VEL);
  setIndexerSafe(-AUTO_INDEXER_VEL);
  setShooterSafe(-AUTO_SHOOTER_VEL);
  while (pros::millis() < endTime)
  {
    pros::delay(10);
  }
}

void discard()
{
  discard(0);
}

void pushAway(int time)
{
  //Run intakes backwards so balls are pushed away and not intaked
  int endTime = pros::millis() + time;
  setIntakesSafe(-AUTO_INTAKE_VEL);
  while (pros::millis() < endTime)
  {
    pros::delay(10);
  }
}

void pushAway()
{
  pushAway(0);
}

void stopMotors()
{
  //Stop everything
  setIntakesSafe(0);
  setIndexerSafe(0);
  setShooterSafe(0);
}
