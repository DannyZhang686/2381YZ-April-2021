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

    auto run = [&, numBallsIn, numBallsOut](void) -> void {
        if ((!setTime) && (initNumBallsShot + numBallsOut <= numBallsShot))
        {
            //Spin the shooter the other way instead, after a short delay
            time = pros::millis();
            setTime = true;
            // s__t(3, "time set");
        }
        else if ((time != 0) && (pros::millis() - time > 50))
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

namespace TurnToPoint
{
    double targetAngle;     //The desired angle, which is constantly updated
    double travellingAngle; //The angle (-π to π) to travel to face the final angle
    double tAngleInches;    //travellingAngle converted to a value in inches (for PD purposes)
    int time;
    bool setTime;
}

double turnCalc(double input)
{
    return pow((std::abs(input) / 127), -0.5) * (input);
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
        rightOutput = -rightTurn.getOutput(0, 20 * tAngleInches);
        // rightOutput = -leftOutput;
        setDriveSafe(leftOutput, rightOutput);
        // Set_Drive(turnCalc(leftOutput), turnCalc(leftOutput), turnCalc(rightOutput), turnCalc(rightOutput));
        s__t(0, "TURN:" + t__s(targetAngle) + " " + t__s(position_tracker->Get_Angle()) + " " + t__s(travellingAngle));
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

namespace TimeBasedMove {
  int counter;
}

AutoTask TimeBasedMoveTask(double velocity, double time) {
  using namespace TimeBasedMove;
  auto init = [&](void) -> void {
    counter = pros::millis();
    setDriveSafe(velocity, velocity);
  };

  auto run = [&, velocity, time]() -> void {
    setDriveSafe(velocity, velocity);
  };

  auto done = [&, time]() -> bool {
      if (pros::millis() - counter > time)
      {
          return true;
      }
      return false;
  };

  auto kill = [] {
    setDriveSafe(0, 0);
  };

  return AutoTask::SyncTask(run, done, init, kill);
}

AutoSequence *Auton::AT_Test_Ultras = AutoSequence::FromTasks(
    vector<AutoTask>{
        //     each tile is 24 inches, (0,0) at center of field, width of bot is 18, length is 14, tracked at center of bot, max distance is 3 tiles (72).
        // autopath(AUTO_DRIVE.CPP) drives to a certain point P {0, -72}, and it will have the angle 0, and reach that point of 127

        SingleRun([](void) -> void {
          position_tracker->Set_Position({36, 12}, PI/2); //Corner first
          // position_tracker->Set_Position({60, 9.5}, 0); //Side first
        }),

        // //Start goal
        // TimeBasedMoveTask(100, 1000).AddInit([](void) -> void {
        //   intakeNoShoot(200);
        // }),
        // IntakeShootTask(0, 1).AddKill([](void) -> void {
        //   stopMotors();
        // }),
        // TimeBasedMoveTask(-65, 650),
        // TurnToPointTask({36, 24}, 0.07),

        //Start to 1
        PurePursuitTask({36, 20.5}, 0, 100).AddInit([](void) -> void {
          intakeNoShoot(200);
        }),
        AutoTask::AutoDelay(250).AddRun([]{
          setDriveSafe(-50, -50);
        }),
        AutoTask::AutoDelay(250).AddRun([]{
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({9, 9}, 0.07),
        AutoTask::AutoDelay(200).AddKill([]{
          stopMotors();
        }),
        PurePursuitTask({9, 9}, 0, 100),

        //Goal 1
        TimeBasedMoveTask(70, 400),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 650),

        //1 to 2
        TurnToPointTask({24, 72}, 0.07),
        PurePursuitTask({24, 72}, 0, 100).AddInit([](void) -> void {
          intakeNoShoot(200);
        }),
        AutoTask::AutoDelay(250).AddRun([]{
          setDriveSafe(-50, -50);
        }),
        AutoTask::AutoDelay(250).AddRun([]{
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({12, 75}, 0.07),
        AutoTask::AutoDelay(200).AddKill([]{
          stopMotors();
        }),
        PurePursuitTask({12, 75}, 0, 100),

        //Goal 2
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 450),

        //2 to 3
        TurnToPointTask({36, 115}, 0.07),
        PurePursuitTask({36, 115}, 0, 100).AddInit([](void) -> void {
          intakeNoShoot(200);
        }),
        AutoTask::AutoDelay(1000).AddRun([]{
          setDriveSafe(-50, -50);
        }),
        AutoTask::AutoDelay(1000).AddRun([]{
          setDriveSafe(0, 0);
        }),
        // TurnToPointTask({18, 114}, 0.07),
        // AutoTask::AutoDelay(200).AddKill([]{
          // stopMotors();
        // }),
        // PurePursuitTask({18, 114}, 0, 100),

        // //Goal 3
        // TimeBasedMoveTask(70, 650),
        // IntakeShootTask(0, 1).AddKill([](void) -> void {
        //   stopMotors();
        // }),
        // TimeBasedMoveTask(-65, 650),

        // //3 to 4
        // TurnToPointTask({72, 96}, 0.07),
        // PurePursuitTask({72, 96}, 0, 100).AddInit([](void) -> void {
        //   intakeNoShoot(200);
        // }).AddKill([](void) -> void {
        //   setDriveSafe(-50, -50);
        //   pros::delay(250);
        //   setDriveSafe(0, 0);
        //   pros::delay(250);
        // }),
        // TurnToPointTask({72, 106}, 0.07),
        // PurePursuitTask({72, 106}, 0, 100).AddInit([](void) -> void {
        //   pros::delay(200);
        //   stopMotors();
        // }),
        //
        // //Goal 4
        // TimeBasedMoveTask(70, 650),
        // IntakeShootTask(0, 1).AddKill([](void) -> void {
        //   stopMotors();
        // }),
        // TimeBasedMoveTask(-65, 650),
        //
        // //4 to 5
        // TurnToPointTask({98, 101}, 0.07),
        // PurePursuitTask({98, 101}, 0, 100).AddInit([](void) -> void {
        //   intakeNoShoot(200);
        // }).AddKill([](void) -> void {
        //   setDriveSafe(-50, -50);
        //   pros::delay(250);
        //   setDriveSafe(0, 0);
        //   pros::delay(250);
        // }),
        // TurnToPointTask({101, 101}, 0.07),
        // PurePursuitTask({101, 101}, 0, 100).AddInit([](void) -> void {
        //   pros::delay(200);
        //   stopMotors();
        // }),
        //
        // //Goal 5
        // TimeBasedMoveTask(70, 650),
        // IntakeShootTask(0, 1).AddKill([](void) -> void {
        //   stopMotors();
        // }),
        // TimeBasedMoveTask(-65, 650),
    });
