
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
    double distance = abs(disp)/3;
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
    newPointList.emplace_back(path[1]);

    // additional points for line 1: solve for linear equation between both points 1 and 2
    // from there, you will get both the slope and y-intercept (which will be common for all points on this line)
    auto Disp2 = path[2] - path[1];
    auto distance2 = abs(Disp2);

    for (double i = 0; i < distance2; i += spacing)
    {
        Pnt newPoint = path[1] + Disp2 * i / distance1;
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

namespace PPS
{

    PointList path = {};

    long mostestClosestIndex = 0;

    Pnt startPoint = 0, endPoint = 0, currentPos = 0, prevPos = 0, waypoint = 0, lookaheadPnt = 0;
    double startAngle = 0, endAngle = 0, currentAngle = 0;

    static double curvature = 1;

    Pd leftStraight1(0.8, 2);
    Pd leftTurn1(0.8, 2);
    Pd rightTurn1(0.8, 2);
    Pd rightStraight1(0.8, 2);

    Pnt current, goalPoint, target, starting;
    Line movingLine;
    double distance, targetAngle; //Distance/Angle between current and target
    double travellingAngle;       //The angle (-π to π, though -π/4 to π/4 works much better) to travel to face the target
    double tAngleInches;          //travellingAngle converted to a value in inches (for PD purposes)
    long long time = 0;
    bool setTime = false;
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

    for (int i = previousIndex; i <  previousIndex + 10; i++)
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
    path = smoother(path, 0.8, 0.2, 12);
    return path;
}

// x, y
// [{0, 1} {0, 2}];
// coord =

//  {a, b}
//  a + bi
// x = a, y = b
// cmplx.real() = a, cmplx.imag() = b
// (0, 1i)
//   Point a = 2 + 3i;
//     Point a = {2,3};

// // motion task

AutoTask PurePursuitTask(complex<double> EndPoint, double EndAngle, array<double, 2> speed, array<double, 2> errorTolerance)
{
    using namespace PPS;
    // run function
    auto runFn = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> void {
        currentPos = position_tracker->Get_Position();
        currentAngle = position_tracker->Get_Angle();
        mostestClosestIndex = GetClosest(path, currentPos, mostestClosestIndex);

        if (mostestClosestIndex >= path.size() - 2)
        {
            Set_Drive(0, 0, 0, 0);
            return;
        }

        waypoint = path[mostestClosestIndex + 1];
        lookaheadPnt = path[mostestClosestIndex + 2];

        auto EndpointDisp = lookaheadPnt - currentPos;
        auto WaypointDisplacement = waypoint - currentPos;

        auto InnerAngle = arg(EndpointDisp) - arg(WaypointDisplacement);

        auto TotalDisp = EndpointDisp;

        // if (cos(InnerAngle) <= 0)
        // {
        //     return;
        // }

        TotalDisp *= sin(InnerAngle) * sin(InnerAngle);
        TotalDisp += (double)2 * WaypointDisplacement * cos(InnerAngle) * cos(InnerAngle) * curvature;

        // auto TargetAngle = EndAngle + pow(cos(InnerAngle), AngleInterpolation) * NormalizeAngle(WaypointAngle - EndAngle);

        // lcd::set_text(3, "DISTANCE: " + to_string(abs(TotalDisp)));
        lcd::set_text(4, "Current: " + to_string(currentPos.real()) + ", " + to_string(currentPos.imag()));
        lcd::set_text(5, "End Point: " + to_string((waypoint).real()) + ", " + to_string((waypoint).imag()));
        // lcd::set_text(4, "Disp: " + to_string(TotalDisp.real()) + ", " + to_string(TotalDisp.imag()));

        auto AngleDisplacement = arg(TotalDisp);
        auto AngleRobot = (position_tracker->Get_Angle());

        auto AngleDiff = remainder(AngleRobot - AngleDisplacement, 2 * M_PI);
        // auto EndAngleDiff = (remainder((AngleRobot - TargetAngle), 2 * M_PI)) / 2;

        // double deaccellCoeff = abs(TotalDisp) * 127 / (12 * speed) < 1 ? abs(TotalDisp) * 127 / (12 * speed) : 1;

        auto Forwards = speed[0] * cos(AngleDiff);
        auto Turn = speed[1] * sin(AngleDiff);
        //  (-0.9 * sin(AngleDiff) / pow(pow(sin(AngleDiff), 2.0), 0.25) + 0.1 * abs(sin(AngleDiff)) / sin(AngleDiff))

        s__t(6, "f: " + t__s(Forwards) + " t: " + t__s(Turn));
                    // lcd::set_text(5, "AngleDiff: " + to_string((int)(AngleDiff*180/M_PI)) + "EndDiff: " + to_string((int)(EndAngleDiff*180/M_PI)));
                    Set_Drive(0, Forwards, Turn, 0);
        // curve to the endpoint and reach that point at angle
    };
    // init function
    auto init = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> void {
        startPoint = currentPos = prevPos = position_tracker->Get_Position();
        startAngle = position_tracker->Get_Angle();
        endPoint = EndPoint;
        endAngle = EndAngle;

        path = GeneratePath(startPoint, endPoint, startAngle, 1);
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