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

        // PurePursuitTask({36, 0}, 0, 127).AddInit(IntakeF(200)),
        // Delay(100000),

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
        // PurePursuitTask({36, 20.5}, 0, 100).AddInit([](void) -> void {
        //   IntakeF(200);
        // }),
        PurePursuitTask({36, 20.5}, 0, 100).AddInit(IntakeF(200)),
        // Delay(200),
        TurnToPointTask({15, 15}, 0.07),
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({15, 15}, 0, 100),

        //Goal 1
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 650),

        //1 to 2
        TurnToPointTask({24, 72}, 0.07),
        PurePursuitTask({24, 72}, 0, 100).AddRun(IntakeF(200)),
        Delay(250).AddRun([]{
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([]{
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({19, 74}, 0.07),
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({19, 74}, 0, 100),

        //Goal 2
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 800),

        //2 to 3
        TurnToPointTask({34, 118}, 0.07),
        PurePursuitTask({34, 118}, 0, 100).AddInit(IntakeF(200)),
        Delay(250).AddRun([] {
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([] {
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({20, 130}, 0.07),
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({20, 130}, 0, 100),

        //Goal 3
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 1000),

        //3 to 4
        TurnToPointTask({70, 101}, 0.07),
        PurePursuitTask({70, 101}, 0, 100).AddInit(IntakeF(200)),
        Delay(250).AddRun([] {
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([] {
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({72, 127}, 0.07),
        PurePursuitTask({72, 127}, 0, 127).AddInit([](void) -> void {
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
        TurnToPointTask({103, 125}, 0.07),
        PurePursuitTask({103, 125}, 0, 100).AddInit(IntakeF(200)),
        Delay(250).AddRun([] {
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([] {
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({126, 132}, 0.07),
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({126, 132}, 0, 100),

        //Goal 5
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 650),

        //////////////////////////////////////////////////////
        // Pure coordinates, no account for centroid of bot //
        //////////////////////////////////////////////////////

        //5 to 6
        TurnToPointTask({121, 75}, 0.07), // Pure: 120, 72
        PurePursuitTask({121, 75}, 0, 100).AddRun(IntakeF(200)), // Pure: 120, 72
        Delay(250).AddRun([]{
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([]{
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({128, 77}, 0.07), // Pure: 144, 72
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({128, 77}, 0, 100), // Pure: 144, 72

        //Goal 6
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 450),

        //6 to 7
        TurnToPointTask({96, 72}, 0.07),
        PurePursuitTask({87, 72}, 0, 100).AddRun(IntakeF(200)), // Pure: 72, 72
        Delay(200).AddKill([] {
          stopMotors();
        }),
        // PurePursuitTask({96, 72}, 0, 100).AddRun(IntakeF(200)),
        // Delay(250).AddRun([]{
        //   setDriveSafe(-50, -50);
        // }),
        // Delay(250).AddRun([]{
        //   setDriveSafe(0, 0);
        // }),
        // TurnToPointTask({88, 72}, 0.07), // Pure: 72, 72

        //Goal 7
        TimeBasedMoveTask(100, 650),
        TimeBasedMoveTask(-65, 400),
        TimeBasedMoveTask(0, 300),
        TimeBasedMoveTask(100, 650),

        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }),
        TimeBasedMoveTask(-65, 650),

        // //7 to 8
        // TurnToPointTask({106, 26}, 0.07), // Pure: 108, 24
        // PurePursuitTask({106, 26}, 0, 100).AddRun(IntakeF(200)), // Pure: 108, 24
        // // Delay(250).AddRun([]{
        // //   setDriveSafe(-50, -50);
        // // }),
        // // Delay(250).AddRun([]{
        // //   setDriveSafe(0, 0);
        // // }),
        // TurnToPointTask({131, 11}, 0.07), // Pure: 144, 0
        //
        // Delay(200).AddKill([] {
        //   stopMotors();
        // }),
        // PurePursuitTask({131, 11}, 0, 100), // Pure: 144, 0
        //
        // //Goal 8
        // TimeBasedMoveTask(70, 650),
        // TimeBasedMoveTask(0, 300),
        //
        // IntakeShootTask(0, 1).AddKill([](void) -> void {
        //   stopMotors();
        // }),
        // TimeBasedMoveTask(-65, 450),
        //
        // //8 to 9
        // TurnToPointTask({72, 54}, 0.07), // Pure: 72, 48
        // PurePursuitTask({72, 54}, 0, 100).AddRun(IntakeF(200)), // Pure: 72, 48
        // // Delay(250).AddRun([]{
        // //   setDriveSafe(-50, -50);
        // // }),
        // // Delay(250).AddRun([]{
        // //   setDriveSafe(0, 0);
        // // }),
        // TurnToPointTask({70, 16}, 0.07), // Pure: 72, 0
        //
        // Delay(200).AddKill([] {
        //   stopMotors();
        // }),
        // PurePursuitTask({72, 16}, 0, 100),
        //
        // //Goal 9
        // TimeBasedMoveTask(70, 650),
        // TimeBasedMoveTask(0, 300),
        //
        // IntakeShootTask(0, 1).AddKill([](void) -> void {
        //   stopMotors();
        // }),
        // TimeBasedMoveTask(-65, 450),

    });
