#include "main.h"

#include "api.h"
#include <math.h>
#include "globals.hpp"
#include "autonomous/auton_control.hpp"
#include "autonomous/auto_timer.hpp"
#include "autonomous/auto_sequence.hpp"
#include <vector>
#include <functional>
#include "autonomous/auto_drive.hpp"
#include "autonomous/global_sequences.hpp"
#include "autonomous.h"
#include "opcontrol.h"
#include "autopathing/pathing.hpp"
#include "pid.h"

using namespace std;
using namespace Auton;
using namespace pros;

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

    auto run = [&, numBallsIn, numBallsOut] (void) -> void
    {
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
    };

    auto done = [&, numBallsIn, numBallsOut](void) -> bool
    {
        if (doneShooting && doneIntaking)
        {
            return true;
        }
        return false;
    };

    auto kill = [](void) -> void
    {
        setIntakesSafe(0);
    };

    return AutoTask::SyncTask(run, done, init, kill);
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

        double leftOutput = 0, rightOutput = 0;                //Power output (0-200) for each side of the robot
        leftOutput = leftTurn.getOutput(0, 20 * tAngleInches); //Setpoint distance value for PD
        rightOutput = -rightTurn.getOutput(0, 20 * tAngleInches);
        
        
        Set_Drive(leftOutput, leftOutput, rightOutput, rightOutput);
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
double turnCalc(double input)
{
    return pow((std::abs(input) / 127), -0.5) * (input);
}

AutoSequence *Auton::AT_Test_Ultras = AutoSequence::FromTasks(
    vector<AutoTask>{
        //     each tile is 24 inches, (0,0) at center of field, width of bot is 18, length is 14, tracked at center of bot, max distance is 3 tiles (72).
        // autopath(AUTO_DRIVE.CPP) drives to a certain point P {0, -72}, and it will have the angle 0, and reach that point of 127

        SingleRun([](void) -> void {
          position_tracker->Set_Position({36, 12}, PI/2);
        }),
        AutoTask::AutoDelay(100),
        PurePursuitTask({36, 30}, 0, 50),
        TurnToPointTask({12, 15}, 0.1),
        PurePursuitTask({12, 15}, 0, 50),
        // SingleRun([](void) -> void { position_tracker->Set_Position({0, 0}, 0, {50, 1}, 0); }),
    });
