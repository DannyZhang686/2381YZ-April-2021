#include <math.h>
#include <list>
#include <functional>

#include "main.h"

#include "api.h"
#include "globals.hpp"
#include "autonomous.h"
#include "opcontrol.h"

#include "autonomous/functional_tasks.hpp"
#include "autonomous/task_lambdas.hpp"
#include "autonomous/global_sequences.hpp"


using namespace std;
using namespace Auton;
using namespace pros;

using namespace TaskLambdas;
AutoSequence Auton::CUS_Q1 = AutoSequence(
  list<AutoTask>({
    SingleRun([](void) -> void {
      position_tracker->Set_Position({92.2-position_tracker->Get_Position().real(), 16.7-position_tracker->Get_Position().imag()}, 0.383972-position_tracker->Get_Angle());
    }),
    Delay(200).AddRun(IntakeF(200)),
    PurePursuitTask({140, 36.5}, 0, 127).AddRun(IntakeF(200)),
    TimeBasedMoveTask(0, 150),
    TimeBasedMoveTask(-75, 600),
    TurnToPointSmooth({132.2, 8}, 0.7, 0.5).AddKill([]{ //119, 10
      stopMotors();
    }),
    PurePursuitTask({132.2, 8}, 0, 80),

    // Goal 1
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(2).TimeLimit(1500), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 400),

    TurnToPointSmooth({90, 81}, 0.7, 0.5).AddRun([]{
      setIntakesSafe(AUTO_INTAKE_VEL);
      setIndexerSafe(-AUTO_INDEXER_VEL);
      setShooterSafe(AUTO_SHOOTER_VEL);
    }),
    PurePursuitTask({90, 81}, 0, 127).AddRun([]{
      discardBack();
    }).AddKill([]{
      stopMotors();
    }),
    TurnToPointSmooth({136, 68.3}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({136, 68.3}, 0, 127),

    // Goal 2
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(1).TimeLimit(1500), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 500),

    TurnToPointSmooth({103.7, 127}, 0.7, 0.5).AddRun([]{
      setIndexerSafe(-AUTO_INDEXER_VEL);
      setShooterSafe(AUTO_SHOOTER_VEL);
    }),
    PurePursuitTask({103.7, 127}, 0, 127).AddRun([]{
      discardBack();
    }).AddKill([]{
      stopMotors();
    }),
    TurnToPointSmooth({144, 106}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({144, 106}, 0, 127),
    TimeBasedMoveTask(0, 150),
    TimeBasedMoveTask(-75, 600),
    TurnToPointSmooth({140, 134}, 0.7, 0.5).AddKill([]{
      stopMotors();
    }),
    PurePursuitTask({140, 134}, 0, 80),

    // Goal 3
    TimeBasedMoveTask(70, 350),
    // Delay(1000),
    IntakeShootTask(2).TimeLimit(1500), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 400),

    TurnToPointSmooth({61, 87}, 0.7, 0.5).AddRun([]{
      setIntakesSafe(AUTO_INTAKE_VEL);
      setIndexerSafe(-AUTO_INDEXER_VEL);
      setShooterSafe(AUTO_SHOOTER_VEL);
    }),
    PurePursuitTask({61, 87}, 0, 127).AddRun([]{
      discardBack();
    }).AddKill([]{
      stopMotors();
    }),
    TurnToPointSmooth({74.8, 135}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({74.8, 135}, 0, 127),

    // Goal 4
    TimeBasedMoveTask(70, 400),
    // Delay(1000),
    IntakeShootTask(1).TimeLimit(1500), //Expecting numBallsInRobot == 1

    // 72, 134
    // SingleRun([](void) -> void {
    //   position_tracker->Set_Position({63-position_tracker->Get_Position().real(), 136-position_tracker->Get_Position().imag()}, (M_PI/2)-position_tracker->Get_Angle());
    // }),
    TimeBasedMoveTask(-70, 500),

    TurnToPointSmooth({26.5, 120}, 0.7, 0.5).AddRun([]{
      setIndexerSafe(-AUTO_INDEXER_VEL);
      setShooterSafe(AUTO_SHOOTER_VEL);
    }),
    PurePursuitTask({26.5, 120}, 0, 127).AddRun([]{
      discardBack();
    }).AddKill([]{
      stopMotors();
    }),
    TurnToPointSmooth({-1, 106}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({-1, 106}, 0, 127),
    TimeBasedMoveTask(0, 150),
    TimeBasedMoveTask(-75, 500),
    TurnToPointSmooth({8.2, 141}, 0.7, 0.5).AddKill([]{
      stopMotors();
    }),
    PurePursuitTask({8.2, 141}, 0, 80),

    // Goal 5
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(2).TimeLimit(1500), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 400),

    TurnToPointSmooth({27, 65}, 0.7, 0.5).AddRun([]{
      setIntakesSafe(AUTO_INTAKE_VEL);
      setIndexerSafe(-AUTO_INDEXER_VEL);
      setShooterSafe(AUTO_SHOOTER_VEL);
    }),
    PurePursuitTask({27, 65}, 0, 127).AddRun([]{
      discardBack();
    }),
    TurnToPointSmooth({0, 74}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({14, 74}, 0, 127),

    // Goal 6
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(1).TimeLimit(1500), //Expecting numBallsInRobot == 1
    TimeBasedMoveTask(-70, 200),

    TurnToPointSmooth({61, 74.5}, 0.7, 0.5).AddRun([]{
      discardFront();
    }),
    PurePursuitTask({61, 74.5}, 0, 127).AddRun(IntakeF(200)),

    // Goal 7 (center)
    Delay(150),
    TimeBasedMoveTask(100, 600),
    Delay(150),
    TimeBasedMoveTask(-40, 200),
    Delay(150),

    TimeBasedMoveTask(100, 400),
    Delay(150),
    TimeBasedMoveTask(-40, 200),
    Delay(150),
    
    TimeBasedMoveTask(100, 500),
    Delay(150),
    IntakeShootTask(0).TimeLimit(1500), //Expecting numBallsInRobot == 1

    TimeBasedMoveTask(-70, 400),
    TurnToPointSmooth({30.5, 19}, 0.7, 0.5),
    PurePursuitTask({30.5, 19}, 0, 127).AddRun(IntakeF(200)),
    TurnToPointSmooth({-1, 46.5}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({-1, 46.5}, 0, 127),
    TimeBasedMoveTask(0, 150),
    TimeBasedMoveTask(-75, 400),
    TurnToPointSmooth({8.5, 11}, 0.7, 0.5),
    PurePursuitTask({8.5, 11}, 0, 127),

    // Goal 8
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(2).TimeLimit(1500), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 400),

    TurnToPointSmooth({78, 53.5}, 0.7, 0.5).AddRun([]{
      setIntakesSafe(AUTO_INTAKE_VEL);
      setIndexerSafe(-AUTO_INDEXER_VEL);
      setShooterSafe(AUTO_SHOOTER_VEL);
    }),
    PurePursuitTask({78, 53.5}, 0, 127).AddRun([]{
      discardBack();
    }).AddKill([]{
      stopMotors();
    }),
    TurnToPointSmooth({73.8, 76.5}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({73.8, 76.5}, 0, 127),
    TimeBasedMoveTask(70, 300),
    IntakeShootTask(0).TimeLimit(1500), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 600),

    // Goal 9
    // TimeBasedMoveTask(70, 300),
    // Delay(1000),
    // IntakeShootTask(1).TimeLimit(1500), //Expecting numBallsInRobot == 1
    // TimeBasedMoveTask(-70, 600),

    // END
    TimeBasedMoveTask(0, 10000000).AddRun([]{
      stopMotors();
    }),
  })
);