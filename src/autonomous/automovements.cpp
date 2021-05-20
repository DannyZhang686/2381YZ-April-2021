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
#include "drive.hpp"

#include "config/profiling.config.hpp"

using namespace std;
using namespace pros;
using namespace std::complex_literals;

// const complex<double> Position_Tracker::wheel_center_offset = {2.75, 2.25};

// step 1 & 2

const double Inner_Product(const Point a, const Point b)
    {
    return a.real() * b.real() + a.imag() * b.imag();
    }


string PositionString(complex<double> point)
    {
    return "x" + t__s(point.real()) + "x" + t__s(point.imag()) + "x";
    }

namespace PPS
    {

    const double maxLookaheadDistance = 18;
    const long lookAheadNumber = 40;
    const double PathSpacing = 1;

    const double deaccelStrength = 3;  // Deacelleration Power - Exponential - 0 for steeper curve at the end, + infinity for steeper curve at the beginning 
    const double turningStrength = 0.1;  // Turning Level Coefficient - Exponential - -1 for max sensitive turning, +1 for max dampened turning 
    const double accelDampening = 0.1; // Top Level Acceleration Control Coefficient - Exponential - Between 0 - 1

            // Remaining Distance Calculator, Range between 0 -1, set to 1 by default in `init`, don't change this.
    const double actualTurningCoeff = exp(turningStrength);

    const double curvature = 1;
    const double H_Wheel_Disp = 6.315;

    }

AutoTask PurePursuitTask(const complex<double> EndPoint, const double EndAngle, const double speed, const double maxAccelCoeff, const double errorTolerance)
    {


    Point* startPoint, * currentPos, * lookaheadPnt, * currentVelocity;

    double* startAngle, * currentAngle, * startDistance, * remainingDistance;

    int* iteration, * mostestClosestIndex, * previousLookaheadIndex;

    array<double, 2>* motorSetpoints;
    PointList* path;

    // run function

    using namespace PPS;

    // init function
    auto init = [&, EndPoint, EndAngle](void) -> void {
        startPoint = new Point(position_tracker->Get_Position());
        currentPos = new Point(position_tracker->Get_Position());
        currentVelocity = new Point(position_tracker->Get_Velocity());
        lookaheadPnt = new Point(EndPoint);

        startAngle = new double(position_tracker->Get_Angle());
        currentAngle = new double(position_tracker->Get_Angle());

        startDistance = new double(abs(EndPoint - (*startPoint)));
        remainingDistance = new double(abs(EndPoint - (*startPoint)));

        path = new PointList(GeneratePath(*startPoint, EndPoint, (*startAngle), PathSpacing));
        motorSetpoints = new array<double, 2>({ 0,0 });

        mostestClosestIndex = new int(0);
        previousLookaheadIndex = new int(0);
        iteration = new int(0);
        };

    auto runFn = [&, EndPoint, EndAngle, speed, maxAccelCoeff](void) -> void {
        try {
            (*currentPos) = (position_tracker->Get_Position());
            (*currentAngle) = position_tracker->Get_Angle();
            (*mostestClosestIndex) = GetClosest((*path), (*currentPos), (*mostestClosestIndex));
            Point currentDisplacement = EndPoint - (*currentPos);

            (*remainingDistance) = abs(currentDisplacement);
            (*currentVelocity) = (position_tracker->Get_Velocity());

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

            auto lookaheadCheck = FindLookAhead((*currentPos), (*path), lookAheadDistance, lookAheadNumber, (*mostestClosestIndex));
            // using mostest closest index as the previous check just to make sure it isn't skipping anything, in case robot is moving like away for example
            // theoretically should store the index of the previous lookahead point if found and use that in place of mostest closest but w/e.

            if (get<1>(lookaheadCheck) != PointNotFound)
                {
                (*lookaheadPnt) = get<1>(lookaheadCheck);
                (*previousLookaheadIndex) = get<0>(lookaheadCheck);
                }
            Point currentHeading = position_tracker->Get_Heading_Vec();

            double lookaheadHeadingComponent = Inner_Product(currentHeading, (*lookaheadPnt) - (*currentPos));

            double forwardsDistance = Inner_Product(currentHeading, currentDisplacement);
            double deaccelSpeed = sqrt(abs(maxAccelCoeff * MAX_MOVE_ACCEL_CONST * forwardsDistance)) * 127 / MAX_MOVE_SPEED_CONST;

            array<double, 2> speeds = { getSignOf(lookaheadHeadingComponent) * min(speed, deaccelSpeed), getSignOf(lookaheadHeadingComponent) * min(speed, deaccelSpeed) };

            double arcRadius = Curvature((*currentPos), (*lookaheadPnt), (*currentAngle));

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

            // s__t(4, "C: [" + t__s(mostestClosestIndex) + "] " + t__s(currentPos.real()) + ", " + t__s(currentPos.imag()));
            // s__t(5, "EP: [" + t__s(previousLookaheadIndex) + "]  " + t__s((lookaheadPnt).real()) + ", " + t__s((lookaheadPnt).imag()));

            *motorSetpoints = Calc_Drive_MPC(speeds[0], speeds[1], true);
            Set_Drive_Direct((*motorSetpoints)[0], (*motorSetpoints)[0], (*motorSetpoints)[1], (*motorSetpoints)[1]);


#define DEBUG
#ifdef DEBUG
            std::string a = "" + t__s(*iteration) + "x" + t__s((*remainingDistance)) + "x" + t__s(abs(*currentVelocity))
                + "x" + t__s((*motorSetpoints)[0]) + "x" + t__s(forwardsDistance) + "x" + t__s(deaccelSpeed)
                // + PositionString(*currentPos)
                + "\n";
            printf(a.c_str());
            (*iteration)++;
#undef DEBUG
#endif
            }
        catch (const std::exception& e)
            {
            printf("ERROR: \n");
            printf(e.what());
            }

        // curve to the endpoint and reach that point at angle
        };

    // done function
    auto done = [&, EndPoint, EndAngle, speed, errorTolerance](void) -> bool {
        bool pastHalfway = (abs(*remainingDistance) < abs(*startDistance) / 2);
        bool withinTolerance = (abs(*remainingDistance) < abs(errorTolerance));
        bool stoppedMoving = abs(*currentVelocity) < 0.05 * MAX_MOVE_SPEED_CONST;
        return (stoppedMoving && (pastHalfway || withinTolerance));
        };

    auto kill = [&](void) -> void {
        delete (path, mostestClosestIndex, previousLookaheadIndex,
            startPoint, currentPos, lookaheadPnt,
            startAngle, currentAngle, startDistance, remainingDistance, currentVelocity,
            iteration, motorSetpoints);
        Set_Drive_Direct(0, 0, 0, 0);
        };

    return AutoTask::SyncTask(
        runFn, done, init, kill);
    }


AutoTask PurePursuitSmooth(Point endPoint, double accel, double errorTolerance)
    {
    PointList* path;

    Point* startPoint, * currentPosition, * lookaheadPnt, * currentVelocity, * headingVector, * currentDisplacement;

    double* currentAngle, * startAngle;

    array<double, 2>* previousSpeeds;

    int* previousLookaheadIndex, * mostestClosestIndex, * iteration;

    double* forwardsDistance, * initialForwardsDistance, * forwardsVelocity;

    using namespace PPS;

    auto init = [&, endPoint, accel, errorTolerance](void) -> void {

        startPoint = new Point(position_tracker->Get_Position());
        currentPosition = new Point(position_tracker->Get_Position());

        // Endpoint + Displacement = Current Position.
        // Vector starting at end point, pointint to current position.
        currentDisplacement = new Point((*currentPosition) - endPoint);


        startAngle = new double(position_tracker->Get_Angle());
        currentAngle = new double(position_tracker->Get_Angle());
        lookaheadPnt = new Point(endPoint);

        previousSpeeds = new array<double, 2>({ 0,0 });
        currentVelocity = new Point(0);
        previousLookaheadIndex = new int(0);

        mostestClosestIndex = new int(0);
        path = new PointList(GeneratePath(*startPoint, endPoint, *startAngle, PathSpacing));


        headingVector = new Point(position_tracker->Get_Heading_Vec());

        initialForwardsDistance = new double(ArcLength(*currentPosition, endPoint, *startAngle));
        forwardsDistance = new double();
        forwardsVelocity = new double();
        iteration = new int(0);
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


        (*headingVector) = (position_tracker->Get_Heading_Vec());

        double innerProduct = Inner_Product((*headingVector), (*lookaheadPnt) - (*currentPosition));
        auto realLookaheadDist = abs((*lookaheadPnt) - (*currentPosition));


        (*currentDisplacement) = endPoint - (*currentPosition);




        // (*forwardsDistance) = Inner_Product(*headingVector, *currentDisplacement);

        (*forwardsDistance) = ArcLength(*currentPosition, endPoint, *currentAngle);

        (*forwardsVelocity) = Inner_Product(*headingVector, *currentVelocity);

        double accelTarget = calcAccelSetpoint(-*forwardsDistance, *forwardsVelocity, errorTolerance, accel * MAX_MOVE_ACCEL_CONST);
        double motorPower = 127 * calcVoltageSetpoint(accelTarget, MOVE_EXPONENT_CONST, *forwardsVelocity, MAX_MOVE_SPEED_CONST);

        array<double, 2> speeds = { motorPower, motorPower };

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
#define DEBUG
#ifdef DEBUG
        std::string a = "" + t__s(*iteration) + "x" + t__s(*forwardsDistance) + "x"
            + t__s(*forwardsVelocity) + "x" + t__s(speeds[0]) + "x" + t__s(speeds[1])
            // + PositionString(currentPosition)
            + "\n";
        printf(a.c_str());
#undef DEBUG
#endif

        // s__t(6, "l: " + t__s(previousSpeeds[0]) + " r: " + t__s(previousSpeeds[1]) + " " + t__s(path.size()));
        Set_Drive_Direct(speeds[0], speeds[0], speeds[1], speeds[1]);
        (*iteration)++;
        };

    auto kill = [&, endPoint, accel, errorTolerance]
        {
        delete (path, startPoint, currentPosition, lookaheadPnt, currentAngle, startAngle, initialForwardsDistance,
            previousSpeeds, previousLookaheadIndex, mostestClosestIndex);
        Set_Drive_Direct(0, 0, 0, 0);
        };

    auto done = [&, endPoint, accel, errorTolerance](void) -> bool
        {
        return (abs(*forwardsDistance) < abs(*initialForwardsDistance) / 2 && abs(*currentVelocity) < 0.05 * MAX_MOVE_SPEED_CONST);
        };

    return AutoTask::SyncTask(run, done, init, kill);
    }



AutoTask PurePursuitSimple(Point endPoint, double accel, double errorTolerance)
    {
    // PointList* path;

    Point* currentPosition, * headingVector, * currentVelocity, * currentDisplacement, * leftEndpoint, * rightEndpoint;

    int* iteration;

    double* forwardsDistance, * initialForwardsDistance, * forwardsVelocity;

    using namespace PPS;

    auto init = [&, endPoint, accel, errorTolerance](void) -> void {

        currentPosition = new Point(position_tracker->Get_Position());

        currentDisplacement = new Point(endPoint - (*currentPosition));

        headingVector = new Point(position_tracker->Get_Heading_Vec());

        initialForwardsDistance = new double(Inner_Product(*headingVector, *currentDisplacement));

        forwardsDistance = new double();
        forwardsVelocity = new double();
        currentVelocity = new Point();

        double currentAngle = position_tracker->Get_Angle();
        double endPointAngOffset = getSignOf(*initialForwardsDistance) * NormalizeAngle(2 * arg(*currentDisplacement) - currentAngle);
        leftEndpoint = new Point(endPoint + Position_Tracker::drive_center_offset * exp<double>(1i * endPointAngOffset)), rightEndpoint = new Point(endPoint - Position_Tracker::drive_center_offset * exp<double>(1i * endPointAngOffset));

        std::string a = t__s((*leftEndpoint).real()) + "x" + t__s(((*leftEndpoint)).imag()) + "x" + t__s((*rightEndpoint).real()) + "x" + t__s(((*rightEndpoint)).imag()) + "\n";
        printf(a.c_str());
        iteration = new int(0);
        };

    auto run = [&, endPoint, accel, errorTolerance]
        {
        (*currentPosition) = position_tracker->Get_Position();
        double currentAngle = position_tracker->Get_Angle();

        (*currentDisplacement) = endPoint - (*currentPosition);
        (*headingVector) = (position_tracker->Get_Heading_Vec());
        (*currentVelocity) = position_tracker->Get_Velocity();

        (*forwardsDistance) = Inner_Product(*headingVector, *currentDisplacement);
        double endPointAngOffset = getSignOf(*forwardsDistance) * NormalizeAngle(2 * arg(*currentDisplacement) - currentAngle);


        Point leftWheelPos = position_tracker->Get_Wheel_Position(Position_Tracker::Wheel_Side::Left), rightWheelPos = position_tracker->Get_Wheel_Position(Position_Tracker::Wheel_Side::Right);


        double driveWheelOffset = abs(Position_Tracker::drive_center_offset);

        double leftDistance = ArcLength((leftWheelPos), (*leftEndpoint), currentAngle);
        double rightDistance = ArcLength((rightWheelPos), (*rightEndpoint), currentAngle);
        double centerDistance = ArcLength((*currentPosition), endPoint, currentAngle);


        // if (abs(leftWheelPos - (*leftEndpoint)) < driveWheelOffset)
        //     {
        //     leftDistance *= abs(leftWheelPos - (*leftEndpoint)) / driveWheelOffset;
        //     }

        // if (abs(rightWheelPos - (*rightEndpoint)) < driveWheelOffset)
        //     {
        //     rightDistance *= abs(rightWheelPos - (*rightEndpoint)) / driveWheelOffset;
        //     }

        (*forwardsVelocity) = Inner_Product(*headingVector, *currentVelocity);

        double midAccelTarget = calcAccelSetpoint(-centerDistance, *forwardsVelocity, errorTolerance, accel * MAX_MOVE_ACCEL_CONST);


        if (abs(centerDistance) < 6)
            {
            centerDistance = getSignOf(centerDistance) * 6;
            }
        double leftAccelTarget = midAccelTarget * leftDistance / centerDistance;
        double rightAccelTarget = midAccelTarget * rightDistance / centerDistance;

        double centerMotorPower = 127 * calcVoltageSetpoint(midAccelTarget, MOVE_EXPONENT_CONST, (*forwardsVelocity), MAX_MOVE_SPEED_CONST);

        double leftWheelSpeed = position_tracker->Get_Wheel_Speed(Position_Tracker::Wheel_Side::Left), rightWheelSpeed = position_tracker->Get_Wheel_Speed(Position_Tracker::Wheel_Side::Right);

        double leftMotorPower = centerMotorPower * leftDistance / centerDistance;
        double rightMotorPower = centerMotorPower * rightDistance / centerDistance;

#define DEBUG
#ifdef DEBUG
        std::string a = "" + t__s(*iteration) + "x" + t__s(leftDistance) + "x" + t__s(rightDistance) + "x" + t__s(leftWheelSpeed)
            + "x" + t__s(rightWheelSpeed) + "x" + t__s(leftAccelTarget) + "x" + t__s(rightAccelTarget) + "\n";
        // std::string a = "" + t__s(*iteration) + "x" + t__s(currentAngle) + "x" + t__s((*currentPosition).real()) + "x" + t__s((leftWheelPos).real())
            // + "x" + t__s((rightWheelPos).real()) + "x" + t__s((leftWheelPos).imag()) + "x" + t__s((rightWheelPos).imag()) + "\n";
        printf(a.c_str());
#undef DEBUG
#endif
        (*iteration)++;
        Set_Drive_Direct(leftMotorPower, leftMotorPower, rightMotorPower, rightMotorPower);
        };

    auto kill = [&, endPoint, accel, errorTolerance]
        {
        delete (currentPosition, headingVector, currentVelocity, currentDisplacement, forwardsDistance, forwardsVelocity, iteration);
        Set_Drive_Direct(0, 0, 0, 0);
        };

    auto done = [&, endPoint, accel, errorTolerance](void) -> bool
        {
        return (abs(*forwardsDistance) < abs(*initialForwardsDistance) / 2 && abs(*currentVelocity) < 0.05 * MAX_MOVE_SPEED_CONST);
        };

    return AutoTask::SyncTask(run, done, init, kill);
    }