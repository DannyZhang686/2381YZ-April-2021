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

using namespace std;
using namespace pros;
using namespace std::complex_literals;

typedef complex<double> Pnt;
typedef vector<complex<double>> PointList;

// const complex<double> Position_Tracker::wheel_center_offset = {2.75, 2.25};

PointList doubleArrayCopy(vector<complex<double>> arr)
{
    //size first dimension of array
    return PointList(arr);
};
// step 1 & 2

PointList DefinePath(Pnt startPoint, Pnt endPoint, double startAngle)
{
    // disp between the 2 points
    Pnt disp = endPoint - startPoint;
    double distance = abs(disp) / 4;
    Pnt midwayPoint = startPoint + distance * Pnt(cos(startAngle), sin(startAngle));

    return {startPoint, midwayPoint, endPoint};
}

PointList InjectPoints(PointList path, double spacing)
{
    // spacing is space between 2 points that r injected
    PointList newPointList = {path[0]};
    // additional points for line 1: solve for linear equation between both points 1 and 2
    // from there, you will get both the slope and y-intercept (which will be common for all points on this line)
    auto Disp = path[1] - path[0];
    auto distance1 = abs(Disp);

    for (double i = 0; i < distance1; i += spacing)
    {
        Pnt newPoint = path[0] + Disp * i / distance1;
        newPointList.emplace_back(newPoint);
    }
    // additional points for line 1: solve for linear equation between both points 1 and 2
    // from there, you will get both the slope and y-intercept (which will be common for all points on this line)
    auto Disp2 = path[2] - path[1];
    auto distance2 = abs(Disp2);

    for (double i = 0; i < distance2; i += spacing)
    {
        Pnt newPoint = path[1] + Disp2 * i / distance2;
        newPointList.emplace_back(newPoint);
    }
    newPointList.emplace_back(path[2]);

    return newPointList;
}

PointList smoother(PointList path, double weight_data, double weight_smooth, double tolerance)
{
    //copy array

    PointList newPath = PointList(path);
    double change = tolerance;

    while (change >= tolerance)
    {
        change = 0.0;

        for (int i = 1; i < path.size() - 1; i++)
        {
            double auxR = newPath[i].real();
            newPath[i] += Pnt((weight_data * (path[i] - newPath[i]) + weight_smooth * (newPath[i - 1] + newPath[i + 1] - (2.0 * newPath[i]))).real(), 0);
            change += abs(auxR - newPath[i].real());

            double auxI = newPath[i].imag();
            newPath[i] += Pnt(0, (weight_data * (path[i] - newPath[i]) + weight_smooth * (newPath[i - 1] + newPath[i + 1] - (2.0 * newPath[i]))).imag());
            change += abs(auxI - newPath[i].imag());
        }
    }

    return newPath;
}

const double Curvature(Pnt currentPos, Pnt targetPos, double currentOrientation)
{
    Pnt disp = targetPos - currentPos;
    double arg = std::arg(disp);

    double dist = abs(targetPos - currentPos);

    if ((arg == currentOrientation))
    {
        return NAN;
    }

    double angleTargetCurrentCenter = arg;
    double angleToTravel = remainder(currentOrientation - angleTargetCurrentCenter, 2 * M_PI);

    //sin(angle) = dist / radius, radius = dist / sin(angle)
    return dist / (2 * sin(angleToTravel));
}

long GetClosest(PointList path, Pnt currentPoint, long previousIndex = 0)
{
    long double min = -1;
    long minIndex = previousIndex;
    previousIndex = previousIndex - 3;
    if (previousIndex < 0)
    {
        previousIndex = 0;
    }

    for (int i = previousIndex; i < previousIndex + 10; i++)
    {
        if (i == path.size() - 1)
        {
            break;
        }
        if (abs(path[i] - currentPoint) < min || min == -1)
        {
            min = abs(path[i] - currentPoint);
            minIndex = i;
        }
    }
    return minIndex;
}

PointList GeneratePath(Pnt startpoint, Pnt endpoint, double startAngle, double spacing)
{
    PointList path = DefinePath(startpoint, endpoint, startAngle);
    path = InjectPoints(path, spacing);
    // Second Picture
    path = smoother(path, 0.5, 0.5, 1);
    return path;
}

static Pnt PointNotFound = Pnt(-100000, 100000);

Pnt CheckIntersection(Pnt circleCenter, Pnt startPoint, Pnt endPoint, double radius)
{
    auto lineSegmentVector = endPoint - startPoint;
    auto circleToStartVector = startPoint - circleCenter;

    double a = abs(lineSegmentVector) * abs(lineSegmentVector);
    double b = 2 * (circleToStartVector.real() * lineSegmentVector.real() + circleToStartVector.imag() * lineSegmentVector.imag());
    double c = abs(circleToStartVector) * abs(circleToStartVector) - radius * radius;
    bool root1Intersect = false, root2Intersect = false;

    double discriminant = b * b - 4 * a * c;

    if (discriminant >= 0)
    {
        discriminant = sqrt(discriminant);

        double root1 = (-b - discriminant) / (2 * a);
        double root2 = (-b + discriminant) / (2 * a);

        // 3x HIT cases:
        //          -o->             --|-->  |            |  --|->
        // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

        // 3x MISS cases:
        //       ->  o                     o ->              | -> |
        // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

        //Between 0 and 1 == between 0% and 100% of the way across the vector
        if ((root2 >= 0) && (root2 <= 1))
        {
            //Return immediately; root2 is further along the vector
            //and so is always a preferable return to root1
            auto a = startPoint + root2 * lineSegmentVector;
            auto dist = abs(a - circleCenter);
            s__t(3, "root2 found: " + t__s(root2) + " dist:" + t__s(dist));
            return startPoint + root2 * lineSegmentVector;
        }
        else if ((root1 >= 0) && (root1 <= 1))
        {
            auto a = startPoint + root1 * lineSegmentVector;
            auto dist = abs(a - circleCenter);
            s__t(3, "root1 found: " + t__s(root2) + " dist:" + t__s(dist));
            return startPoint + root1 * lineSegmentVector;
        }
    }
    return PointNotFound;
}


namespace PPS
{

    PointList path = {};

    long mostestClosestIndex = 0;

    Pnt startPoint = 0, endPoint = 0, currentPos = 0, prevPos = 0, lookaheadPnt = 0;
    double startAngle = 0, endAngle = 0, currentAngle = 0;

    long previousLookaheadIndex = 0;

    static double lookAheadDistance = 10;
    static long lookAheadNumber = 40;
    static double PathSpacing = 1;

    static double curvature = 1;
    static double H_Wheel_Disp = 6.315;

}

tuple<long, Pnt> FindLookAhead(Pnt currentPos, PointList path, double radius, long previousIndex = 0)
{
    long index = previousIndex;
    long size = path.size();

    while (index < previousIndex + PPS::lookAheadNumber)
    {
        if (index == size - 2)
        {
            return {size - 2, path[size - 1]};
        };
        Pnt lookaheadCheck = CheckIntersection(currentPos, path[index], path[index + 1], radius);
        if (lookaheadCheck != PointNotFound)
        {
            return {index, lookaheadCheck};
        }
        index++;
    }

    s__t(3, "sadge: " + t__s(size) + " " + t__s(previousIndex));
    return {previousIndex, PointNotFound};
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
        if (mostestClosestIndex >= path.size())
        {
            Set_Drive(0, 0, 0, 0);
            return;
        }

        auto lookaheadCheck = FindLookAhead(currentPos, path, lookAheadDistance, mostestClosestIndex);
        // using mostest closest index as the previous check just to make sure it isn't skipping anything, in case robot is moving like away for example
        // theoretically should store the index of the previous lookahead point if found and use that in place of mostest closest but w/e.

        if (get<1>(lookaheadCheck) != PointNotFound)
        {
            lookaheadPnt = get<1>(lookaheadCheck);
            previousLookaheadIndex = get<0>(lookaheadCheck);
        }

        array<double, 2> speeds = {speed, speed};

        double arcRadius = Curvature(currentPos, lookaheadPnt, currentAngle);

        if (arcRadius != NAN || arcRadius != INFINITY)
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
                speeds[1] = speeds[1] * rightRadius / leftRadius;
            }
            else
            {
                speeds[0] = speeds[0] * leftRadius / rightRadius;
            }
        }

        // lcd::set_text(3, "DISTANCE: " + to_string(abs(TotalDisp)));
        lcd::set_text(4, "Current: " + to_string(currentPos.real()) + ", " + to_string(currentPos.imag()));
        lcd::set_text(5, "End Point: " + to_string((lookaheadPnt).real()) + ", " + to_string((lookaheadPnt).imag()));
        // lcd::set_text(4, "Disp: " + to_string(TotalDisp.real()) + ", " + to_string(TotalDisp.imag()));

        s__t(6, "l: " + t__s(speeds[0]) + " r: " + t__s(speeds[1]));
        // lcd::set_text(5, "AngleDiff: " + to_string((int)(AngleDiff*180/M_PI)) + "EndDiff: " + to_string((int)(EndAngleDiff*180/M_PI)));
        Set_Drive(speeds[0], speeds[0], speeds[1], speeds[1]);
        // curve to the endpoint and reach that point at angle
    };
    // init function
    auto init = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> void {
        startPoint = currentPos = prevPos = position_tracker->Get_Position();
        startAngle = position_tracker->Get_Angle();
        endPoint = EndPoint;
        endAngle = EndAngle;

        lookaheadPnt = EndPoint;
        previousLookaheadIndex = 0;
        path = GeneratePath(startPoint, endPoint, startAngle, PathSpacing);
    };
    // done function
    auto done = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> bool {
        return mostestClosestIndex >= path.size() - 2;
    };

    auto kill = [](void) -> void { Set_Drive(0, 0, 0, 0); };

    return AutoTask::SyncTask(
        runFn, done, init, kill);
}

// void Set_Path_Drive(complex<double> EndPoint, double EndAngle, array<double, 2> speed, array<double, 2> errorTolerance)
// {
//     auto currentPos = position_tracker->Get_Position_N();
//     auto Displacement = EndPoint - CurrentPos;

//     lcd::set_text(3, "DISTANCE: " + to_string(abs(Displacement)));
//     // lcd::set_text(4, "Current: " + to_string(CurrentPos.real()) + ", " + to_string(CurrentPos.imag()));
//     // lcd::set_text(5, "End Pnt: " + to_string(EndPoint.real()) + ", " + to_string(EndPoint.imag()));
//     lcd::set_text(4, "Disp: " + to_string(Displacement.real()) + ", " + to_string(Displacement.imag()));

//     auto AngleDisplacement = arg(Displacement);
//     auto AngleRobot = (position_tracker->Get_Angle());

//     auto AngleDiff = remainder(AngleRobot - AngleDisplacement, 2 * M_PI);
//     //Normalize turn angle to between (-pi and pi) so sin(x/2) works properly

//     auto EndAngleDiff = (remainder((AngleRobot - EndAngle), 2 * M_PI)) / 2;

//     if (abs(Displacement) < 1 * errorTolerance[0] && abs(EndAngleDiff) < 0.01 * errorTolerance[1])
//     {
//         Stop();
//         _is_running = false;
//         return;
//     }
//     if (abs(position_tracker->Get_Velocity()) < 0.1 && abs(position_tracker->Get_Ang_Vel()) < 0.1)
//     {
//         lcd::set_text(3, "STUCK");
//         stuck += 1;
//         if (stuck > 40)
//         {
//             Stop();
//             _is_running = false;
//             return;
//         }
//     }
//     else
//     {
//         stuck = 0;
//     }

//     lcd::set_text(3, "Vel" + to_string(abs(position_tracker->Get_Velocity()) < 0.1) + " Ang Vel + " + to_string(abs(position_tracker->Get_Ang_Vel()) < 0.1));

//     double deaccellCoeff = abs(Displacement) * 127 / (9 * speed[0]) < 1 ? abs(Displacement) * 127 / (9 * speed[0]) : 1;
//     double deaccelSecond = abs(Displacement) * 127 / (6 * speed[0]) < 1 ? abs(Displacement) * 127 / (6 * speed[0]) : 1;

//     // double deaccellCoeff = 1;

//     // TurnControl->Update(0, sin(EndAngleDiff));
//     auto Forwards = speed[0] * cos(AngleDiff) * deaccellCoeff;
//     auto Strafe = speed[0] * sin(AngleDiff) * deaccellCoeff;

//     auto Turna = (0.9 * sin(EndAngleDiff) / pow(pow(sin(EndAngleDiff), 2.0), 0.25) + 0.15 * abs(sin(EndAngleDiff)) / sin(EndAngleDiff));

//     auto Turn = speed[1] * ((abs(EndAngleDiff) < 0.5 * errorTolerance[1] || deaccellCoeff / errorTolerance[1] > 1) ? Turna : Turna * deaccellCoeff + (EndAngleDiff / abs(EndAngleDiff)) * (1 - deaccellCoeff));

//     lcd::set_text(5, "AngleDiff: " + to_string((int)(AngleDiff * 180 / M_PI)) + "EndDiff: " + to_string((int)(EndAngleDiff * 180 / M_PI)));
//     // lcd::set_text(6, "Input: " + to_string((int)Strafe) + " / " + to_string((int)Forwards) + " / " + to_string((int)Turn));
//     Set_Drive(Strafe, Forwards, Turn, 0);
// }