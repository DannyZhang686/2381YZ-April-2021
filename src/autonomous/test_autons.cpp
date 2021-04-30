#include "main.h"

#include "api.h"
#include <math.h>
#include "globals.hpp"
#include <vector>
#include <functional>
#include "autonomous.h"
#include "opcontrol.h"
#include "autonomous/functional_tasks.hpp"

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
  int runNumber = 0;
  auto PrintLocation(std::string text)
  {
    return [&, text] {
      std::string a = "(" + text + ") RUN: " + t__s(runNumber) + " pos: " + t__s(position_tracker->Get_Position().real()) + ", " + t__s(position_tracker->Get_Position().imag()) + " \n";
      printf(a.c_str());
      runNumber++;
    };
  }
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
    return Delay(time).AddRun([velocity] { setDriveSafe(velocity, velocity); }).AddKill(SetDrive0).AddKill(PrintLocation(""));
  };

}

using namespace TaskLambdas;

AutoSequence* Auton::AT_Test_Ultras = AutoSequence::FromTasks(
  vector<AutoTask>({

    TurnToPointSMOOTH({-50, 50}, 100, 1),



    // DriveProfileTask(80),
    RestProfileTask(),
    Delay(100000),
      TurnToPointTask({-50, 50}, 0.07), // Changed from {25, 78} to 26, 77  because getting stuck at second tower on turn

      // Delay(100000),

      SingleRun([](void) -> void {
      // position_tracker->Set_Position({36, 12}, PI / 2);  //UNCOMMENT TO CHANGE
      position_tracker->Set_Position({56, 13.5}, 0); //COMMENT TO CHANGE
    }),

    //Start goal
    TimeBasedMoveTask(100, 650),                     //COMMENT TO CHANGE
    IntakeShootTask(0, 1).AddKill([](void) -> void { //COMMENT TO CHANGE
      stopMotors();                                  //COMMENT TO CHANGE
    }).TimeLimit(1000),                              //COMMENT TO CHANGE
    TimeBasedMoveTask(-65, 500),                     //COMMENT TO CHANGE
    TurnToPointTask({16, 34}, 0.07),                 //COMMENT TO CHANGE

    //Start to 1
    // PurePursuitTask({36, 24}, 0, 100).AddInit(IntakeF(200)),   //UNCOMMENT TO CHANGE
    PurePursuitTask({16, 34}, 0 , 80).AddInit(IntakeF(200)).AddKill(PrintLocation("ball on wall")), //COMMENT TO CHANGE
    TimeBasedMoveTask(0, 200),
    TimeBasedMoveTask(-70, 650), //COMMENT TO CHANGE
    // RUN 4

    SingleRun([](void) -> void {
      position_tracker->Set_Position({-1, 4}, 0); //COMMENT TO CHANGE
    }),

    TurnToPointTask({16, 22.5}, 0.07).AddKill(PrintLocation("turn to goal")).AddKill([] { //From {16, 24} -> {16, 22}
      stopMotors();
    }),
    Delay(200),
    PurePursuitTask({16, 22}, 0, 70).AddKill(PrintLocation("go to goal")), //COMMENT TO CHANGE
    //RUN 6
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
    TimeBasedMoveTask(-65, 1000), //Time from 650 to 1k

    //1 to 2
    TurnToPointTask({30.5, 77}, 0.07), // Changed from {25, 78} to 26, 77  because getting stuck at second tower on turn
    PurePursuitTask({30.5, 77}, 0, 127).AddRun(IntakeF(200)).AddKill(PrintLocation("go to second goal ball")),

    Delay(250),



      TurnToPointTask({15, 75}, 0.07).AddRun([] {
        stopMotors();
      }),

      TimeBasedMoveTask(50, 1000),
      IntakeShootTask(0, 1).AddKill([](void) -> void {
        stopMotors();
      }).TimeLimit(1000),
      TimeBasedMoveTask(-65, 800),

        //2 to 3
        TurnToPointTask({41, 124}, 0.07),
        PurePursuitTask({41, 124}, 0, 127).AddInit(IntakeF(200)).AddKill(PrintLocation("go to third goal ball")),

        Delay(250),

        TurnToPointTask({20, 128}, 0.07),
        PurePursuitTask({20, 128}, 0, 60).AddKill([] {
          stopMotors();
        }).AddKill(PrintLocation("go to third goal")),

          //Goal 3
          TimeBasedMoveTask(70, 650),
          IntakeShootTask(0, 1).AddKill([](void) -> void {
            stopMotors();
          }).TimeLimit(1000),
          TimeBasedMoveTask(-65, 1000),

            //3 to 4
            SingleRun([](void) -> void {
            // position_tracker->Set_Position({36, 12}, PI / 2);  //UNCOMMENT TO CHANGE
            position_tracker->Set_Position({0, 4}, 0); //COMMENT TO CHANGE
          }),

          TurnToPointTask({76, 100}, 0.07),
          PurePursuitTask({76, 100}, 0, 127).AddInit(IntakeF(200)),

          Delay(250),

          TurnToPointTask({76, 133}, 0.07),
          PurePursuitTask({76, 125}, 0, 100).AddRun([] { // 136 before
            stopMotors();
          }),

            //Goal 4
            TimeBasedMoveTask(40, 650),
            IntakeShootTask(0, 1).AddKill([](void) -> void {
              stopMotors();
            }).TimeLimit(1000),
            TimeBasedMoveTask(-65, 1000),

              //4 to 5
              TurnToPointTask({110, 123.5}, 0.07),
              PurePursuitTask({110, 123.5}, 0, 127).AddInit(IntakeF(200)),
              //No delay; keep momentum instead

              TurnToPointTask({126, 134}, 0.07), // From 126 , 136
              PurePursuitTask({126, 134}, 0, 60).AddRun([] {
                stopMotors();
              }),

              //Goal 5
              TimeBasedMoveTask(70, 650),
              IntakeShootTask(0, 1).AddKill([](void) -> void {
                stopMotors();
              }).TimeLimit(1000),
              TimeBasedMoveTask(-65, 650),

                //5 to 6
                TurnToPointTask({116, 77}, 0.07),                        // Pure: 120, 72
                PurePursuitTask({116, 77}, 0, 127).AddRun(IntakeF(200)), // Pure: 120, 72

                Delay(250),

                TurnToPointTask({134, 75}, 0.07).AddKill([] { // Pure: 144, 72
                  stopMotors();
                }),

                //Goal 6
                TimeBasedMoveTask(70, 1000),
                IntakeShootTask(0, 1).AddKill([](void) -> void {
                  stopMotors();
                }).TimeLimit(1000),
                TimeBasedMoveTask(-65, 650),

                  //6 to 7
                  TurnToPointTask({87, 81}, 0.07),
                  PurePursuitTask({87, 81}, 0, 100).AddRun(IntakeF(200)), //Pure: 72, 72

                  //Goal 7
                  // TimeBasedMoveTask(100, 650).AddKill([] {
                  //   stopMotors();
                  // }),
                  TimeBasedMoveTask(-70, 400),
                  TimeBasedMoveTask(0, 200),
                  TimeBasedMoveTask(127, 400),
                  TimeBasedMoveTask(-70, 400),
                  TimeBasedMoveTask(0, 200),
                  TimeBasedMoveTask(127, 400),
                  TimeBasedMoveTask(-70, 400),
                  TimeBasedMoveTask(0, 200),
                  TimeBasedMoveTask(127, 400),

                  IntakeShootTask(0, 1).AddKill([](void) -> void {
                    stopMotors();
                  }).TimeLimit(1000),
                  TimeBasedMoveTask(-127, 650),

                    //7 to 8
                    TurnToPointTask({100, 36}, 0.07),                        // Pure: 108, 24
                    PurePursuitTask({100, 36}, 0, 100).AddRun(IntakeF(200)), // Pure: 108, 24

                    Delay(250),

                    TurnToPointTask({123, 25}, 0.07), // Pure: 144, 0
                    PurePursuitTask({123, 25}, 0, 100).AddRun([] { // Pure: 144, 0
                      stopMotors();
                    }),

                    //Goal 8
                    TimeBasedMoveTask(70, 650),
                    IntakeShootTask(0, 1).AddKill([](void) -> void {
                      stopMotors();
                    }).TimeLimit(1000),
                    TimeBasedMoveTask(-65, 1000),
    })
);