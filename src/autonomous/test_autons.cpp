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
          // position_tracker->Set_Position({36, 12}, PI / 2);  //UNCOMMENT TO CHANGE
          position_tracker->Set_Position({56, 13.5}, 0);          //COMMENT TO CHANGE
        }),

        //Start goal
        TimeBasedMoveTask(100, 650),                            //COMMENT TO CHANGE
        IntakeShootTask(0, 1).AddKill([](void) -> void {        //COMMENT TO CHANGE
          stopMotors();                                         //COMMENT TO CHANGE
        }),                                                     //COMMENT TO CHANGE
        TimeBasedMoveTask(-65, 500),                            //COMMENT TO CHANGE
        TurnToPointTask({10, 39}, 0.07),                        //COMMENT TO CHANGE

        //Start to 1
        // PurePursuitTask({36, 24}, 0, 100).AddInit(IntakeF(200)),   //UNCOMMENT TO CHANGE
        PurePursuitTask({10, 39}, 0, 100).AddInit(IntakeF(200)),      //COMMENT TO CHANGE
        TimeBasedMoveTask(-70, 650),                                  //COMMENT TO CHANGE
        TurnToPointTask({15, 24}, 0.07),                               //COMMENT TO CHANGE
        Delay(200).AddKill([] {                                       //COMMENT TO CHANGE
          stopMotors();                                               //COMMENT TO CHANGE
        }),                                                          //COMMENT TO CHANGE
        PurePursuitTask({15, 24}, 0, 100),                            //COMMENT TO CHANGE
        // TurnToPointTask({15, 15}, 0.07),                          //UNCOMMENT TO CHANGE
        // Delay(200).AddKill([] {                                    //UNCOMMENT TO CHANGE
        //   stopMotors();                                           //UNCOMMENT TO CHANGE
        // }),                                                       //UNCOMMENT TO CHANGE
        // PurePursuitTask({15, 15}, 0, 100),                       //UNCOMMENT TO CHANGE

        //Goal 1
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }).TimeLimit(1000),
        TimeBasedMoveTask(-65, 650),

        //1 to 2
        TurnToPointTask({25, 78}, 0.07),
        PurePursuitTask({25, 78}, 0, 100).AddRun(IntakeF(200)),
        Delay(250).AddRun([]{
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([]{
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({20, 76}, 0.07),
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({20, 76}, 0, 100),

        //Goal 2
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }).TimeLimit(1000),
        TimeBasedMoveTask(-65, 800),

        //2 to 3
        TurnToPointTask({37, 128}, 0.07),
        PurePursuitTask({37, 128}, 0, 100).AddInit(IntakeF(200)),
        Delay(250).AddRun([] {
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([] {
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({22, 135}, 0.07),
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({22, 135}, 0, 100),

        //Goal 3
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }).TimeLimit(1000),
        TimeBasedMoveTask(-65, 1000),

        //3 to 4
        TurnToPointTask({78, 106}, 0.07),
        PurePursuitTask({78, 106}, 0, 100).AddInit(IntakeF(200)),
        Delay(250).AddRun([] {
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([] {
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({78, 136}, 0.07),
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({78, 136}, 0, 127),

        //Goal 4
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }).TimeLimit(1000),
        TimeBasedMoveTask(-65, 650),

        //4 to 5
        TurnToPointTask({110, 126}, 0.07),
        PurePursuitTask({110, 126}, 0, 100).AddInit(IntakeF(200)),
        Delay(250).AddRun([] {
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([] {
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({126, 136}, 0.07),
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({126, 136}, 0, 100),

        //Goal 5
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }).TimeLimit(1000),
        TimeBasedMoveTask(-65, 650),

        //////////////////////////////////////////////////////
        // Pure coordinates, no account for centroid of bot //
        //////////////////////////////////////////////////////

        //5 to 6
        TurnToPointTask({125, 72}, 0.07), // Pure: 120, 72
        PurePursuitTask({125, 72}, 0, 100).AddRun(IntakeF(200)), // Pure: 120, 72
        Delay(250).AddRun([]{
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([]{
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({128, 80}, 0.07), // Pure: 144, 72
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({128, 80}, 0, 100), // Pure: 144, 72

        //Goal 6
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }).TimeLimit(1000),
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
        TimeBasedMoveTask(-65, 400),
        TimeBasedMoveTask(0, 300),
        TimeBasedMoveTask(100, 650),
        TimeBasedMoveTask(-65, 400),
        TimeBasedMoveTask(0, 300),
        TimeBasedMoveTask(100, 650),

        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }).TimeLimit(1000),
        TimeBasedMoveTask(-65, 650),

        //7 to 8
        TurnToPointTask({108, 36}, 0.07), // Pure: 108, 24
        PurePursuitTask({108, 36}, 0, 100).AddRun(IntakeF(200)), // Pure: 108, 24
        Delay(250).AddRun([]{
          setDriveSafe(-50, -50);
        }),
        Delay(250).AddRun([]{
          setDriveSafe(0, 0);
        }),
        TurnToPointTask({131, 23}, 0.07), // Pure: 144, 0
        Delay(200).AddKill([] {
          stopMotors();
        }),
        PurePursuitTask({131, 23}, 0, 100), // Pure: 144, 0

        //Goal 8
        TimeBasedMoveTask(70, 650),
        IntakeShootTask(0, 1).AddKill([](void) -> void {
          stopMotors();
        }).TimeLimit(1000),
        TimeBasedMoveTask(-65, 650),

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
