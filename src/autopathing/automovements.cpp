#include "main.h"
#include "opcontrol.h"
#include "motors.h"
#include "utilities.h"
#include "control/motor_controller.hpp"

#include "autonomous.h"
#include "utilities.h"
#include "pid.h"
#include "autonomous/auto_task.hpp"

#include "control/pid.hpp"
#include "globals.hpp"
#include <math.h> /* pow */
#include <numeric>
#include <complex>
#include <array>
#include <cmath>
#include <vector>

#include "pathing.hpp"

using namespace std;
using namespace pros;
using namespace std::complex_literals;

// const complex<double> Position_Tracker::wheel_center_offset = {2.75, 2.25};

// step 1 & 2

double powCalc(double x, double power)
{
    return pow(abs(x), power) * x;
}


namespace PPS
{

    PointList path = {};

    long mostestClosestIndex = 0;

    Point startPoint = 0, endPoint = 0, currentPos = 0, prevPos = 0, lookaheadPnt = 0;
    double startAngle = 0, endAngle = 0, currentAngle = 0;

    long previousLookaheadIndex = 0;

    static double lookAheadDistance = 15;
    static long lookAheadNumber = 40;
    static double PathSpacing = 1;

    static double turningStrength = 0.5; //
    static double actualTurningCoeff = exp(turningStrength) - 1;

    static double curvature = 1;
    static double H_Wheel_Disp = 6.315;

}

tuple<long, Point> FindLookAhead(Point currentPos, PointList path, double radius, long previousIndex = 0)
{
    long index = previousIndex;
    long size = path.size();

    while (index < previousIndex + PPS::lookAheadNumber)
    {
        if (index >= size - 2)
        {
            s__t(3, "end of line");
            return {size - 1, path[size - 1]};
        };
        Point lookaheadCheck = CheckIntersection(currentPos, path[index], path[index + 1], radius);
        if (lookaheadCheck != PointNotFound)
        {
            return {index, lookaheadCheck};
        }
        index++;
    }

    s__t(3, "sadge: " + t__s(size) + " " + t__s(previousIndex));
    return {previousIndex, PointNotFound};
}

double inner_product(Point a, Point b)
{
    return a.real() * b.real() + a.imag() * b.imag();
}
int getSignOf(double yeet)
{
    if (yeet >= 0)
    {
        return 1;
    }
    return -1;
}
AutoTask PurePursuitTask(complex<double> EndPoint, double EndAngle, double speed, array<double, 2> errorTolerance)
{
    using namespace PPS;
    // run function
    auto runFn = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> void {
        currentPos = position_tracker->Get_Position();
        currentAngle = position_tracker->Get_Angle();
        mostestClosestIndex = GetClosest(path, currentPos, mostestClosestIndex);

        /**
         * TODO:
         * 1. ADJUSTABLE TURNING STRENGTH (TURNING COEFFICIENT) ^ (TURNING STRENGTH)
         * 2. ACCEL AND DEACCEL CURVES - DEPENDING ON CURRENT SPEED, MOSTEST CLOSEST INDEX
         * 3. PATHING - JUST LIKE WAY BETTER PATH GENERATION, PATH VISUALIZATION - WHAT DOES PATH SMOOTHING EVEN DO
         * 4. END ANGLE RESILIENCE TESTING - IF YOU *NEED* TO END AT SOME ANGLE IN FRONT OF GOAL, HOW CONSISTENT IS THAT
         * 5. INTELLIGENT PATH *O FUCK* RECOGNITION - IF ITS WAY OFF THE PATH, GENERATION NEW PATH FROM CURRENT POINT TO END POINT?
         *  - MAYBE CONTINUALLY ADJUST THE PATH BASED ON THAT?
         *
         *
         * ACTUALLY MAYBE EVERYTHING LISTED ABOVE ISN'T WORTH AND SHOULD JUST SPEND 4 DAYS SPAM TESTING CONSISTENCY?
         * CONSISTENCY CONSISTENCY CONSISTENCY
         *
         *
         * **/

        auto lookaheadCheck = FindLookAhead(currentPos, path, lookAheadDistance, mostestClosestIndex);
        // using mostest closest index as the previous check just to make sure it isn't skipping anything, in case robot is moving like away for example
        // theoretically should store the index of the previous lookahead point if found and use that in place of mostest closest but w/e.

        if (get<1>(lookaheadCheck) != PointNotFound)
        {
            lookaheadPnt = get<1>(lookaheadCheck);
            previousLookaheadIndex = get<0>(lookaheadCheck);
        }
        Point currentHeading = exp<double>(1i * currentAngle);

        double innerProduct = inner_product(currentHeading, lookaheadPnt - currentPos);

        auto realLookaheadDist = abs(lookaheadPnt - currentPos);

        auto deaccelCoeff = 0.5 + 0.5 * min(realLookaheadDist/lookAheadDistance, 1.0);

        array<double, 2> speeds = {getSignOf(innerProduct) * speed * deaccelCoeff, getSignOf(innerProduct) * speed * deaccelCoeff};

        double arcRadius = Curvature(currentPos, lookaheadPnt, currentAngle);

        if (arcRadius != NAN && arcRadius != INFINITY)
        {
            double leftRadius = arcRadius + H_Wheel_Disp;
            double rightRadius = arcRadius - H_Wheel_Disp;

            if (abs(leftRadius) == 0)
            {
                speeds[0] = 0;
            }
            else if (abs(rightRadius) == 0)
            {
                speeds[1] = 0;
            }
            else if (abs(leftRadius) > abs(rightRadius))
            {
                speeds[1] = speeds[1] * powCalc(rightRadius / leftRadius, actualTurningCoeff);
            }
            else
            {
                speeds[0] = speeds[0] * powCalc(leftRadius / rightRadius, actualTurningCoeff);
            }
        }

        // lcd::set_text(3, "DISTANCE: " + to_string(abs(TotalDisp)));
        lcd::set_text(4, "C: [" +  t__s(mostestClosestIndex) +  "] " + to_string(currentPos.real()) + ", " + to_string(currentPos.imag()));
        lcd::set_text(5, "EP: ["  + t__s(previousLookaheadIndex) + "]  " + to_string((lookaheadPnt).real()) + ", " + to_string((lookaheadPnt).imag()));
        // lcd::set_text(4, "Disp: " + to_string(TotalDisp.real()) + ", " + to_string(TotalDisp.imag()));

        s__t(6, "l: " + t__s(speeds[0]) + " r: " + t__s(speeds[1]) + " " + t__s(path.size()));
        // lcd::set_text(5, "AngleDiff: " + to_string((int)(AngleDiff*180/M_PI)) + "EndDiff: " + to_string((int)(EndAngleDiff*180/M_PI)));
        Set_Drive(speeds[0], speeds[0], speeds[1], speeds[1]);
        // curve to the endpoint and reach that point at angle
    };
    // init function
    auto init = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> void {
        startPoint = currentPos = position_tracker->Get_Position();
        startAngle = position_tracker->Get_Angle();
        endPoint = EndPoint;
        endAngle = EndAngle;

        lookaheadPnt = EndPoint;
        previousLookaheadIndex = 0;
        mostestClosestIndex = 0;
        path = GeneratePath(startPoint, endPoint, startAngle, PathSpacing);
    };
    // done function
    auto done = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> bool {
        return mostestClosestIndex >= (path.size() - 2);
    };

    auto kill = [](void) -> void { Set_Drive(0, 0, 0, 0); };

    return AutoTask::SyncTask(
        runFn, done, init, kill);
}

// namespace TurnToPoint
// {
//     double targetAngle;     //The desired angle, which is constantly updated
//     double travellingAngle; //The angle (-π to π) to travel to face the final angle
//     double tAngleInches;    //travellingAngle converted to a value in inches (for PD purposes)
//     int time;
//     bool setTime;
// }

// AutoTask TurnToPointTask(Point target, double maxError)
// {
//     using namespace TurnToPoint;

//     auto init = [&](void) -> void {
//         time = 0;
//         setTime = false;
//     };

//     auto runFn = [&, target, maxError]() -> void {
//         targetAngle = arg(position_tracker->Get_Position() - target);

//         travellingAngle = smallestAngle(robotPos.angle, targetAngle);
//         tAngleInches = angleToInches(travellingAngle);

//         double leftOutput = 0, rightOutput = 0;                //Power output (0-200) for each side of the robot
//         leftOutput = leftTurn.getOutput(0, 20 * tAngleInches); //Setpoint distance value for PD
//         rightOutput = -rightTurn.getOutput(0, 20 * tAngleInches);
//         Set_Drive(leftOutput, leftOutput, rightOutput, rightOutput);
//         s__t(4, t__s(targetAngle) + " " + t__s(robotPos.angle) + " " + t__s(travellingAngle));
//     };

//     auto doneFn = [&, maxError]() -> bool {
//         if ((!setTime) && (fabs(travellingAngle) < maxError))
//         {
//             time = pros::millis();
//             setTime = true;
//         }
//         else if ((time != 0) && (pros::millis() - time > 250))
//         {
//             return true;
//         }
//         return false;
//     };
//     auto kill = [] {
//         Set_Drive(0, 0, 0, 0);
//     };
//     return AutoTask::SyncTask(runFn, doneFn, init, kill);
// }
