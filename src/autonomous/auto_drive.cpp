#include "globals.hpp"
#include "pathing.hpp"
#include "autonomous.h"
#include "pid.h"
#include "opcontrol.h"

double turnCalc(double input)
{
    return pow((std::abs(input) / 127), -0.5) * (input);
}

using namespace std;

AutoTask SingleRun(std::function<void(void)> run)
{
    return AutoTask(run, [](void) -> bool { return true; });
}

namespace TurnToPoint
{
    double targetAngle;     //The desired angle, which is constantly updated
    double travellingAngle; //The angle (-π to π) to travel to face the final angle
    double tAngleInches;    //travellingAngle converted to a value in inches (for PD purposes)
    int time;
    bool setTime;
}

AutoTask TurnToPointTask(Point target, double maxError)
{
    using namespace TurnToPoint;

    auto init = [&](void) -> void {
        time = 0;
        setTime = false;
    };

    auto runFn = [&, target, maxError]() -> void {
        targetAngle = arg(target - position_tracker->Get_Position());

        travellingAngle = -smallestAngle(position_tracker->Get_Angle(), targetAngle);
        tAngleInches = angleToInches(travellingAngle);

        double leftOutput = 0, rightOutput = 0; //Power output (0-200) for each side of the robot
        // leftOutput = turnCalc(leftTurn.getOutput(0, 20 * tAngleInches)); //Setpoint distance value for PD
        leftOutput = leftTurn.getOutput(0, 20 * tAngleInches);
        rightOutput = -rightTurn.getOutput(0, 20 * tAngleInches);

        // rightOutput = -leftOutput;
        setDriveSafe(leftOutput, rightOutput);
        // Set_Drive(turnCalc(leftOutput), turnCalc(leftOutput), turnCalc(rightOutput), turnCalc(rightOutput));
        s__t(0, "TURN:" + t__s(targetAngle) + " " + t__s(position_tracker->Get_Angle()) + " " + t__s(travellingAngle));
        s__t(1, "TURN_VEL:" + t__s(leftOutput) + " " + t__s(rightOutput));
    };

    auto doneFn = [&, maxError]() -> bool {
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
    auto kill = [] {
        Set_Drive(0, 0, 0, 0);
    };
    return AutoTask::SyncTask(runFn, doneFn, init, kill);
}

AutoTask TurnToAngleTask(double heading, double maxError)
{
    using namespace TurnToPoint;

    auto init = [&](void) -> void {
        time = 0;
        setTime = false;
    };

    auto runFn = [&, heading, maxError]() -> void {
        targetAngle = heading;

        travellingAngle = -smallestAngle(position_tracker->Get_Angle(), targetAngle);
        tAngleInches = angleToInches(travellingAngle);

        double leftOutput = 0, rightOutput = 0; //Power output (0-200) for each side of the robot
        // leftOutput = turnCalc(leftTurn.getOutput(0, 20 * tAngleInches)); //Setpoint distance value for PD
        rightOutput = -rightTurn.getOutput(0, 20 * tAngleInches);
        // rightOutput = -leftOutput;
        setDriveSafe(-rightOutput, rightOutput);
        // Set_Drive(leftOutput, leftOutput, -leftOutput, -leftOutput);
        s__t(4, t__s(targetAngle) + " " + t__s(position_tracker->Get_Angle()) + " " + t__s(travellingAngle));
    };

    auto doneFn = [&, maxError]() -> bool {
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
    auto kill = [] {
        Set_Drive(0, 0, 0, 0);
    };
    return AutoTask::SyncTask(runFn, doneFn, init, kill);
}

namespace IntakeShoot
{
    double initNumBallsShot;    //Number of balls shot before this point
    double initNumBallsIntaken; //Number of balls intaken before this point
    bool doneIntaking, doneShooting;
    int time;
    bool setTime;
}

AutoTask IntakeShootTask(int numBallsIn, int numBallsOut)
{
    using namespace IntakeShoot;
    auto init = [&, numBallsIn, numBallsOut](void) -> void {
        initNumBallsShot = numBallsShot;       //Number of balls shot before this point
        initNumBallsIntaken = numBallsIntaken; //Number of balls intaken before this point
        doneIntaking = false, doneShooting = false;
        time = 0;
        setTime = false;
        setIntakesSafe(AUTO_INTAKE_VEL);
        setIndexerSafe(AUTO_INDEXER_VEL);
        setShooterSafe(AUTO_SHOOTER_VEL);
    };

    auto run = [&, numBallsIn, numBallsOut](void) -> void {
        if ((!setTime) && (initNumBallsShot + numBallsOut <= numBallsShot))
        {
            //Spin the shooter the other way instead, after a short delay
            time = pros::millis();
            setTime = true;
            // s__t(3, "time set");
        }
        else if ((time != 0) && (pros::millis() - time > 0))
        {
            setShooterSafe(-AUTO_SHOOTER_VEL);
            doneShooting = true;
            // s__t(4, "");
        }
        if ((numBallsIn == 0) || (initNumBallsIntaken + numBallsIn + 0.5 <= numBallsIntaken))
        {
            //Stop the intakes
            setIntakesSafe(0);
            doneIntaking = true;
        }
    };

    auto done = [&, numBallsIn, numBallsOut](void) -> bool {
        if (doneShooting && doneIntaking)
        {
            return true;
        }
        return false;
    };

    auto kill = [](void) -> void {
        setIntakesSafe(0);
    };

    return AutoTask::SyncTask(run, done, init, kill);
}
