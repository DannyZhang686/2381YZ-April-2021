#include "main.h"

#include "api.h"
#include <math.h>
#include "globals.hpp"
#include <vector>
#include <functional>
#include "autonomous.h"
#include "opcontrol.h"

using namespace std;
using namespace Auton;
using namespace pros;

namespace TaskLambdas
{

  /**
 * @brief Intake Single Run Lambda - Previously Intake No Shoot
 * @param intakeSpeed Intake Voltage default 200
 * 
 */

  auto IntakeF(double intakeSpeed = 200)
  {
    return [intakeSpeed] {
      intakeNoShoot(intakeSpeed);
    };
  };

  /**
* @brief Lambda Sets Drive To 0 With Both Safe and PID Drive
**/
  auto SetDrive0 = [] {
    setDriveSafe(0, 0);
    Set_Drive(0, 0, 0, 0);
  };

  // Uses `SetDriveSafe(velocity, velocity)` for `time` ms duration, then SetDrive0
  const AutoTask TimeBasedMoveTask(double velocity, double time)
  {
    return Delay(time).AddRun([velocity] { setDriveSafe(velocity, velocity); }).AddKill(SetDrive0);
  };
}

using namespace TaskLambdas;

AutoSequence *Auton::AT_Test_Ultras = AutoSequence::FromTasks(
    vector<AutoTask>{
        //     each tile is 24 inches, (0,0) at center of field, width of bot is 18, length is 14, tracked at center of bot, max distance is 3 tiles (72).
        // autopath(AUTO_DRIVE.CPP) drives to a certain point P {0, -72}, and it will have the angle 0, and reach that point of 127

        SingleRun([](void) -> void {
          position_tracker->Set_Position({36, 12}, PI / 2); //Corner first
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
        PurePursuitTask({36, 20.5}, 0, 100).AddInit(IntakeF(200)),
        Delay(200),
        TurnToPointTask({9, 9}, 0.07),
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({9, 9}, 0, 100),

        //Goal 1
        TimeBasedMoveTask(70, 400),
        TimeBasedMoveTask(0, 300),

        // IntakeShootTask(0, 1).AddKill([](void) -> void {
        //   stopMotors();
        // }),
        TimeBasedMoveTask(-65, 650),

        //1 to 2
        TurnToPointTask({24, 72}, 0.07),
        PurePursuitTask({24, 72}, 0, 100).AddRun(IntakeF(200)),
        // Delay(250).AddRun([]{
        //   setDriveSafe(-50, -50);
        // }),
        // Delay(250).AddRun([]{
        //   setDriveSafe(0, 0);
        // }),
        TurnToPointTask({12, 72}, 0.07),

        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({12, 72}, 0, 100),

        //Goal 2
        TimeBasedMoveTask(70, 650),
        TimeBasedMoveTask(0, 300),

        // IntakeShootTask(0, 1).AddKill([](void) -> void {
        //   stopMotors();
        // }),
        TimeBasedMoveTask(-65, 450),

        //2 to 3
        TurnToPointTask({36, 115}, 0.07),

        PurePursuitTask({36, 115}, 0, 100).AddInit(IntakeF(200)),

// NEW STUFF - Added after merge starts here

        // Delay(250).AddRun([] {
        //   setDriveSafe(-50, -50);
        // }),
        // Delay(250).AddRun([] {
        //   setDriveSafe(0, 0);
        // }), - Removed This Because Deaccel should compensate
        TurnToPointTask({13, 125}, 0.07),
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({13, 125}, 0, 100),

        //Goal 3
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 650),

        //3 to 4
        TurnToPointTask({72, 90}, 0.07),


        // FIXME - UHH CAN WE NOT HAVE PROS DELAY INSIDE TASK BODY

        PurePursuitTask({72, 90}, 0, 100).AddInit(IntakeF(200))
            .AddKill([](void) -> void {
              setDriveSafe(-50, -50);
              pros::delay(250);
              setDriveSafe(0, 0);
              pros::delay(250);
            }),
        TurnToPointTask({74, 120}, 0.07),
        PurePursuitTask({74, 120}, 0, 100).AddInit([](void) -> void {
          pros::delay(200);
          stopMotors();
        }),

        //Goal 4
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 650),

        //4 to 5
        TurnToPointTask({108, 105}, 0.07),
        PurePursuitTask({108, 105}, 0, 100).AddInit([](void) -> void {
                                             intakeNoShoot(200);
                                           })
            .AddKill([](void) -> void {
              setDriveSafe(-50, -50);
              pros::delay(250);
              setDriveSafe(0, 0);
              pros::delay(250);
            }),
        TurnToPointTask({136, 125}, 0.07),
        PurePursuitTask({136, 125}, 0, 100).AddInit([](void) -> void {
          pros::delay(200);
          stopMotors();
        }),

        //Goal 5
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 650),
    });
