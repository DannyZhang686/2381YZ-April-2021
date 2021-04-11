#include "main.h"
#include "autonomous.h"
#include "motors.h"
#include "utilities.h"
#include "pid.h"
#include "autonomous/auto_task.hpp"

// namespace PPS
// {

//     Pd leftStraight1(0.8, 2);
//     Pd leftTurn1(0.8, 2);
//     Pd rightTurn1(0.8, 2);
//     Pd rightStraight1(0.8, 2);

//     Point current, goalPoint, target, starting;
//     Line movingLine;
//     double distance, targetAngle; //Distance/Angle between current and target
//     double travellingAngle;       //The angle (-π to π, though -π/4 to π/4 works much better) to travel to face the target
//     double tAngleInches;          //travellingAngle converted to a value in inches (for PD purposes)
//     long long time = 0;
//     bool setTime = false;
// }

// AutoTask PurePursuitTask(double targetX, double targetY, double maxError, double lookDist, bool goForward)
// {
//     using namespace PPS;
//     auto runFn = [&, lookDist, goForward](void) -> void {
//         current.setValues(robotPos.x, robotPos.y);
//         goalPoint = movingLine.findGoalPoint(current, lookDist);

//         // distance = findDistance(current, goalPoint);
//         targetAngle = findAngle(current, goalPoint);
//         if (goForward)
//         {
//             travellingAngle = smallestAngle(robotPos.angle, targetAngle);
//         }
//         else
//         {
//             travellingAngle = smallestAngle(rotatePi(robotPos.angle), targetAngle);
//         }
//         tAngleInches = angleToInches(travellingAngle);

//         double leftOutput = 0, rightOutput = 0; //Power output (0-200) for each side of the robot
//         //Calculate new drive values, adding together linear and angular values for a final number
//         // leftOutput += leftStraight1.getOutput(0, 15*distance); //Call to PID to find velocity
//         // rightOutput += rightStraight1.getOutput(0, 15*distance);
//         leftOutput += leftStraight1.getOutput(15 * findDistance(starting, current), 15 * distance); //Call to PID to find velocity
//         rightOutput += rightStraight1.getOutput(15 * findDistance(starting, current), 15 * distance);
//         if (time == 0)
//         {
//             leftOutput += 1 * leftTurn1.getOutput(0, 15 * tAngleInches); //Setpoint distance value for PD
//             rightOutput -= 1 * rightTurn1.getOutput(0, 15 * tAngleInches);
//         }
//         if (!goForward)
//         {
//             leftOutput = -leftOutput;
//             rightOutput = -rightOutput;
//         }
//         s__t(2, "PPS Outputs" + t__s(leftOutput) + " " + t__s(rightOutput));
//         s__t(3, "dist, pos" + t__s(findDistance(current, target)) + " " + t__s(current.x) + " " + t__s(current.y));
//         s__t(4, "gp" + t__s(goalPoint.x) + " " + t__s(goalPoint.y));

//         setDriveSafe(leftOutput, rightOutput);
//     };

//     auto init = [&, targetX, targetY](void) -> void {
//         leftStraight1.resetError();
//         leftTurn1.resetError();
//         rightStraight1.resetError();
//         rightTurn1.resetError();

//         current = Point(robotPos.x, robotPos.y); //Current coordinates as per tracking
//         target = Point(targetX, targetY);        //Target coordinates
//         movingLine = Line(current, target);      //Line between starting and target coordinates
//         time = 0;
//         setTime = false;
//         starting = Point(robotPos.x, robotPos.y);
//         distance = findDistance(starting, target);
//     };
//     auto done = [&, maxError](void) -> bool {
//         if ((!setTime) && (findDistance(current, target) < maxError))
//         {

//             time = pros::millis();
//             setTime = true;
//         }
//         if ((time != 0) && (pros::millis() - time > 300))
//         {
//             return true;
//         }
//         return false;
//     };
//     auto kill = [](void) -> void { setDriveSafe(0, 0); };

//     return AutoTask::SyncTask(
//         runFn, done, init, kill);
// }

namespace TTP
{
    double targetAngle;     //The desired angle, which is constantly updated
    double travellingAngle; //The angle (-π to π) to travel to face the final angle
    double tAngleInches;    //travellingAngle converted to a value in inches (for PD purposes)
    long long time = 0;
    bool setTime = false;

    Pd leftTurn1(0.8, 2);
    Pd rightTurn1(0.8, 2);
}

AutoTask TurnToPointTask(double targetX, double targetY, double maxError)
{
    using namespace TTP;
    auto init = [&](void) -> void {
        leftTurn1.resetError();
        rightTurn1.resetError();
        time = 0;
        setTime = false;
    };

    auto runFn = [&, targetX, targetY](void) -> void {
        targetAngle = findAngle(Point(robotPos.x, robotPos.y), Point(targetX, targetY));
        travellingAngle = smallestAngle(robotPos.angle, targetAngle);
        tAngleInches = angleToInches(travellingAngle);

        double leftOutput = 0, rightOutput = 0;                      //Power output (0-200) for each side of the robot
        leftOutput = TTP::leftTurn1.getOutput(0, 25 * tAngleInches); //Setpoint distance value for PD
        rightOutput = -TTP::rightTurn1.getOutput(0, 25 * tAngleInches);
        setDriveSafe(leftOutput, rightOutput);
        s__t(2, "TTP Outputs" + t__s(leftOutput) + " " + t__s(rightOutput));
        s__t(4, t__s(targetAngle) + " " + t__s(robotPos.angle) + " " + t__s(travellingAngle));
    };

    auto done = [&, maxError](void) -> bool {
        s__t(3, "TTP " + t__s(maxError - fabs(travellingAngle)) + " " + t__s(time));
        if ((!setTime) && (fabs(travellingAngle) < maxError))
        {
            time = pros::millis();
            setTime = true;
        }
        else if ((time != 0) && (pros::millis() - time > 250))
        {
            return true;
        }
        return false;
    };
    auto kill = [](void) -> void { setDriveSafe(0, 0); };

    return AutoTask::SyncTask(
        runFn, done, init, kill);
}

AutoTask ApproachGoalTask(double vel, double t)
{
    return AutoTask::AutoDelay(t).AddRun([vel] {
                                     s__t(2, "AG Outputs");
                                     setDriveSafe(vel, vel);
                                 })
        .AddKill([] {
            setDriveSafe(0, 0);
        });
};