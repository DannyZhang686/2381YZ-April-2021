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
#include <queue>

#include "autonomous/pathing.hpp"
#include "autonomous/functional_tasks.hpp"

using namespace std;
using namespace pros;
using namespace std::complex_literals;

// const complex<double> Position_Tracker::wheel_center_offset = {2.75, 2.25};

// step 1 & 2

double powCalc(double x, double power)
{
    return pow(abs(x), power) * getSignOf(x);
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

namespace PPS
{

    const double maxLookaheadDistance = 18;
    const long lookAheadNumber = 40;
    const double startDistance = 0;
    const double PathSpacing = 1;

    const double deaccelStrength = 3;  // Deacelleration Power - Exponential - 0 for steeper curve at the end, + infinity for steeper curve at the beginning 
    const double turningStrength = 0.1;  // Turning Level Coefficient - Exponential - -1 for max sensitive turning, +1 for max dampened turning 
    const double accelDampening = 0.1; // Top Level Acceleration Control Coefficient - Exponential - Between 0 - 1

            // Remaining Distance Calculator, Range between 0 -1, set to 1 by default in `init`, don't change this.
    const double actualTurningCoeff = exp(turningStrength);

    const double curvature = 1;
    const double H_Wheel_Disp = 6.315;

}

AutoTask PurePursuitTask(complex<double> EndPoint, double EndAngle, double speed, array<double, 2> errorTolerance)
{


    PointList& path = *(new PointList());

    long& mostestClosestIndex = *(new long(0)), & previousLookaheadIndex = *(new long(0));

    Point& startPoint = *(new Point(0)), & endPoint = *(new Point(0)), & currentPos = *(new Point(0)), & prevPos = *(new Point(0)),
        & lookaheadPnt = *(new Point(0));

    double& startAngle = *(new double(0)), & endAngle = *(new double(0)),
        & currentAngle = *(new double(0)), & startDistance = *(new double(0)), & deaccelCoeff = *(new double(0));

    int& iteration = *(new int(0));

    array<double, 2>& previousSpeeds = *(new array<double, 2>({0, 0}));
    // run function

    using namespace PPS;

    auto runFn = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> void {
        try {
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

            double lookAheadDistance = maxLookaheadDistance * speed / 127;

            auto lookaheadCheck = FindLookAhead(currentPos, path, lookAheadDistance, lookAheadNumber, mostestClosestIndex);
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

            deaccelCoeff = pow(min(realLookaheadDist / min(lookAheadDistance, startDistance / 2), 1.0), deaccelStrength);

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

            previousSpeeds = {previousSpeeds[0] * accelDampening + (1 - accelDampening) * speeds[0], previousSpeeds[1] * accelDampening + (1 - accelDampening) * speeds[1]};

            s__t(4, "C: [" + t__s(mostestClosestIndex) + "] " + t__s(currentPos.real()) + ", " + t__s(currentPos.imag()));
            s__t(5, "EP: [" + t__s(previousLookaheadIndex) + "]  " + t__s((lookaheadPnt).real()) + ", " + t__s((lookaheadPnt).imag()));
            std::string a = "" + t__s(iteration) + "x" + t__s(realLookaheadDist) + "x" + t__s(previousSpeeds[0]) + "x" + t__s(currentPos.real()) + "\n";
            printf(a.c_str());
            (iteration)++;

            s__t(6, "l: " + t__s(previousSpeeds[0]) + " r: " + t__s(previousSpeeds[1]) + " " + t__s(path.size()));
            Set_Drive(previousSpeeds[0], previousSpeeds[0], previousSpeeds[1], previousSpeeds[1]);
        }
        catch (const std::exception& e)
        {
            printf("ERROR: \n");
            printf(e.what());
        }

        // curve to the endpoint and reach that point at angle
    };
    // init function
    auto init = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> void {
        startPoint = currentPos = position_tracker->Get_Position();
        startAngle = position_tracker->Get_Angle();
        endPoint = lookaheadPnt = EndPoint;
        endAngle = EndAngle;
        startDistance = abs(endPoint - startPoint);

        previousSpeeds = {0,0};
        previousLookaheadIndex = 0;
        mostestClosestIndex = 0;
        deaccelCoeff = 1;
        path = GeneratePath(startPoint, endPoint, startAngle, PathSpacing);
    };
    // done function
    auto done = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> bool {
        auto a = (deaccelCoeff < 0.5 && (abs(previousSpeeds[0]) + abs(previousSpeeds[1])) < 40);
        if (a) printf("finished");
        return a;
    };

    auto kill = [&](void) -> void {

        s__t(0, "pps finished");
        delete (&mostestClosestIndex, &previousLookaheadIndex, &startPoint, &endPoint,
            &currentPos, &prevPos, &lookaheadPnt, &startAngle, &endAngle, &currentAngle,
            &startDistance, &deaccelCoeff);
        Set_Drive(0, 0, 0, 0);
    };

    return AutoTask::SyncTask(
        runFn, done, init, kill);
}

#define ROBOT_L
// #define ROBOT_Z


#ifdef ROBOT_L
const double MAX_MOVE_SPEED_CONST = 12 * M_PI / 180; // 0.191986217719
const double MAX_MOVE_ACCEL_CONST = 1 * M_PI / 180; // 0.00872664625997
const double MOVE_EXPONENT_CONST = -0.1135;
#undef ROBOT_L
#endif


#ifdef ROBOT_Z
const double MAX_MOVE_SPEED_CONST = 12 * M_PI / 180; // 0.191986217719
const double MAX_MOVE_ACCEL_CONST = 1 * M_PI / 180; // 0.00872664625997
const double MOVE_EXPONENT_CONST = -0.1135;
#undef ROBOT_Z
#endif

AutoTask PurePursuitSmooth(Point endPoint, double accel, double errorTolerance)
{
    PointList* path;

    Point* startPoint, * currentPosition, * lookaheadPnt, *currentVelocity;

    double* currentAngle, * startAngle, * startDistance, * deaccelCoeff;

    array<double, 2>* previousSpeeds;

    int* previousLookaheadIndex, * mostestClosestIndex;

    using namespace PPS;

    auto init = [&, endPoint, accel, errorTolerance](void) -> void {

        startPoint = new Point(position_tracker->Get_Position());
        currentPosition = new Point(position_tracker->Get_Position());
        startAngle = new double(position_tracker->Get_Angle());
        lookaheadPnt = new Point(endPoint);

        startDistance = new double(abs(endPoint - *startPoint));
        previousSpeeds = new array<double, 2>({0,0});
        currentVelocity = new Point(0);
        previousLookaheadIndex = new int(0);

        mostestClosestIndex = new int(0);
        deaccelCoeff = new double(1);
        path = new PointList(GeneratePath(*startPoint, endPoint, *startAngle, PathSpacing));
    };

    auto run = [&, endPoint, accel, errorTolerance]
    {
        (*currentPosition) = (position_tracker->Get_Position());
        (*currentAngle) = position_tracker->Get_Angle();
        (*mostestClosestIndex) = GetClosest(*path, *currentPosition, *mostestClosestIndex);
        (*currentVelocity) = position_tracker->Get_Velocity();

        double lookAheadDistance = maxLookaheadDistance * abs(*currentVelocity) / MAX_MOVE_SPEED_CONST;

        auto lookaheadCheck = FindLookAhead((*currentPosition), *path, lookAheadDistance, lookAheadNumber, *mostestClosestIndex);
            // using mostest closest index as the previous check just to make sure it isn't skipping anything, in case robot is moving like away for example
            // theoretically should store the index of the previous lookahead point if found and use that in place of mostest closest but w/e.

        if (get<1>(lookaheadCheck) != PointNotFound)
        {
            *lookaheadPnt = get<1>(lookaheadCheck);
            *previousLookaheadIndex = get<0>(lookaheadCheck);
        }
        Point currentHeading = exp<double>(1i * (*currentAngle));

            double innerProduct = inner_product(currentHeading, (*lookaheadPnt) - (*currentPosition));
            auto realLookaheadDist = abs((*lookaheadPnt) - (*currentPosition));

            // deaccelCoeff = pow(min(realLookaheadDist / min(lookAheadDistance, startDistance / 2), 1.0), deaccelStrength);

            array<double, 2> speeds = {getSignOf(innerProduct), getSignOf(innerProduct)};

            double arcRadius = Curvature((*currentPosition), (*lookaheadPnt), (*currentAngle));

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

            // previousSpeeds = {previousSpeeds[0] * accelDampening + (1 - accelDampening) * speeds[0], previousSpeeds[1] * accelDampening + (1 - accelDampening) * speeds[1]};

            // s__t(4, "C: [" + t__s(mostestClosestIndex) + "] " + t__s(currentPos.real()) + ", " + t__s(currentPos.imag()));
            // s__t(5, "EP: [" + t__s(previousLookaheadIndex) + "]  " + t__s((lookaheadPnt).real()) + ", " + t__s((lookaheadPnt).imag()));
            // std::string a = "" + t__s(iteration) + "x" + t__s(realLookaheadDist) + "x" + t__s(previousSpeeds[0]) + "x" + t__s(currentPos.real()) + "\n";
            // printf(a.c_str());
            // (iteration)++;

            // s__t(6, "l: " + t__s(previousSpeeds[0]) + " r: " + t__s(previousSpeeds[1]) + " " + t__s(path.size()));
            Set_Drive_Direct(0,0,0,0);

    };

    auto kill = [&, endPoint, accel, errorTolerance]
    {
        delete (path, startPoint, currentPosition, lookaheadPnt, currentAngle, startAngle, startDistance,
            deaccelCoeff, previousSpeeds, previousLookaheadIndex, mostestClosestIndex);
        Set_Drive(0, 0, 0, 0);
    };

    auto done = [&, endPoint, accel, errorTolerance](void) -> bool
    {

    };

    return AutoTask::SyncTask(run, done, init, kill);
}