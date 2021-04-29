#include "globals.hpp"
#include "pathing.hpp"
#include "autonomous.h"
#include "pid.h"
#include "opcontrol.h"
#include "utilities.h"
#include "motors.h"
using namespace std;

AutoTask SingleRun(std::function<void(void)> run)
{
    return AutoTask(run, [](void) -> bool { return true; });
}
namespace DriveProfile
{
    double targetAngle;  //The desired angle, which is constantly updated
    double angleDiff;    //The angle (-π to π) to travel to face the final angle
    double tAngleInches; //travellingAngle converted to a value in inches (for PD purposes)
    int time;
    bool setTime;

    static double turnCoeff = 0.95;
}

AutoTask DriveProfileTask(double speed)
{
    int* iteration;
    auto init = [&](void)->void
    {
        iteration = new int(0);
    };

    auto run = [&, speed](void) -> void 
    {
        Set_Drive_Direct(speed, speed, -speed, -speed);
        double currentSpeed = abs(position_tracker->Get_Velocity());
        double currentAngVel = position_tracker->Get_Ang_Vel()*180/M_PI;

		double currentWheelSpeed = (abs(leftBack->get_actual_velocity()) +abs( rightBack->get_actual_velocity()) +abs( leftFront->get_actual_velocity()) + abs(rightFront->get_actual_velocity()))/4;
        s__t(3, t__s(*iteration));
        std::string a = "" + t__s(*iteration) + "x"  + t__s(currentAngVel) + "\n";
        std::string b = "" + t__s(*iteration) + "x"  + t__s(currentWheelSpeed) + ")\n";

        printf(a.c_str());
        printf(b.c_str());

        (*iteration)++;
    };

    auto done = [&](void) -> bool
    {
        return false;
    };

    auto kill = [&](void)->void
    {
        delete iteration;
    };

    return AutoTask::SyncTask(run, done, init, kill);

}
namespace TurnToPoint
{
    double targetAngle;  //The desired angle, which is constantly updated
    double angleDiff;    //The angle (-π to π) to travel to face the final angle
    double tAngleInches; //travellingAngle converted to a value in inches (for PD purposes)
    int time;
    bool setTime;

    static double turnCoeff = 0.95;
}

AutoTask TurnToPointTask(Point target, double maxError, double turnSpeed)
{
    using namespace TurnToPoint;

    auto init = [&](void) -> void {
	  time = 0;
        setTime = false;
    };

    auto runFn = [&, target, maxError, turnSpeed]() -> void {
        targetAngle = arg(target - position_tracker->Get_Position());

        angleDiff = NormalizeAngle(targetAngle - position_tracker->Get_Angle());

    
        auto input = (abs(angleDiff) > M_PI/2) ? turnSpeed * getSignOf(angleDiff) :  turnSpeed * (turnCoeff * sin(angleDiff) / pow(pow(sin(angleDiff), 2.0), -0.1) + (1 - turnCoeff) * abs(sin(angleDiff)) / sin(angleDiff));

        
        Set_Drive(-input, -input, input, input);
        s__t(0, "TURN:" + t__s(targetAngle) + " " + t__s(position_tracker->Get_Angle()) + " " + t__s(angleDiff));
        s__t(1, "TURN_VEL:" + t__s(input) + " " +  t__s(position_tracker->Get_Ang_Vel()));
    };

    auto doneFn = [&, maxError]() -> bool {
        return (abs(angleDiff)  < maxError && abs(position_tracker->Get_Ang_Vel()) < maxError);
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
        // targetAngle = heading;

        // travellingAngle = -smallestAngle(position_tracker->Get_Angle(), targetAngle);
        // tAngleInches = angleToInches(travellingAngle);

        // double leftOutput = 0, rightOutput = 0; //Power output (0-200) for each side of the robot
        // // leftOutput = turnCalc(leftTurn.getOutput(0, 20 * tAngleInches)); //Setpoint distance value for PD
        // rightOutput = -rightTurn.getOutput(0, 20 * tAngleInches);
        // // rightOutput = -leftOutput;
        // setDriveSafe(-rightOutput, rightOutput);
        // // Set_Drive(leftOutput, leftOutput, -leftOutput, -leftOutput);
        // s__t(4, t__s(targetAngle) + " " + t__s(position_tracker->Get_Angle()) + " " + t__s(travellingAngle));
    };

    auto doneFn = [&, maxError]() -> bool {
        // if ((!setTime) && (fabs(travellingAngle) < maxError))
        // {
        //     time = pros::millis();
        //     setTime = true;
        // }
        // else if ((time != 0) && (pros::millis() - time > 250))
        // {
        //     return true;
        // }
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
