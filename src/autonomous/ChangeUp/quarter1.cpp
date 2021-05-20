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
    PurePursuitTask({140, 37}, 0, 127).AddRun(IntakeF(200)),
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

    TurnToPointSmooth({94.6, 81.5}, 0.7, 0.5).AddRun([]{
      setIntakesSafe(AUTO_INTAKE_VEL);
      setIndexerSafe(-AUTO_INDEXER_VEL);
      setShooterSafe(AUTO_SHOOTER_VEL);
    }),
    PurePursuitTask({94.6, 81.5}, 0, 127).AddRun([]{
      discardBack();
    }),
    TurnToPointSmooth({136, 70.5}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({136, 70.5}, 0, 127),

    // Goal 2
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(1).TimeLimit(1500), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 400),

    TurnToPointSmooth({110.7, 127}, 0.7, 0.5),
    PurePursuitTask({110.7, 127}, 0, 127).AddRun([]{
      discardBack();
    }),
    TurnToPointSmooth({140, 105}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({140, 105}, 0, 127),
    TimeBasedMoveTask(0, 150),
    TimeBasedMoveTask(-75, 600),
    TurnToPointSmooth({136, 133}, 0.7, 0.5).AddKill([]{ //119, 10
      stopMotors();
    }),
    PurePursuitTask({136, 133}, 0, 80),

    // Goal 3
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(2).TimeLimit(1500), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 400),

    TurnToPointSmooth({62, 97}, 0.7, 0.5).AddRun([]{
      setIntakesSafe(AUTO_INTAKE_VEL);
      setIndexerSafe(-AUTO_INDEXER_VEL);
      setShooterSafe(AUTO_SHOOTER_VEL);
    }),
    PurePursuitTask({62, 97}, 0, 127).AddRun([]{
      discardBack();
    }),
    TurnToPointSmooth({62, 133}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({62, 133}, 0, 127),

    // Goal 4
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(1).TimeLimit(1500), //Expecting numBallsInRobot == 1
    TimeBasedMoveTask(-70, 400),

    TurnToPointSmooth({25, 127}, 0.7, 0.5),
    PurePursuitTask({25, 127}, 0, 127).AddRun([]{
      discardBack();
    }),
    TurnToPointSmooth({1, 108}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({1, 108}, 0, 127),
    TimeBasedMoveTask(0, 150),
    TimeBasedMoveTask(-75, 600),
    TurnToPointSmooth({6, 138}, 0.7, 0.5).AddKill([]{ //119, 10
      stopMotors();
    }),
    PurePursuitTask({6, 138}, 0, 80),

    // Goal 5
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(2).TimeLimit(1500), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 400),

    TurnToPointSmooth({20, 65}, 0.7, 0.5).AddRun([]{
      setIntakesSafe(AUTO_INTAKE_VEL);
      setIndexerSafe(-AUTO_INDEXER_VEL);
      setShooterSafe(AUTO_SHOOTER_VEL);
    }),
    PurePursuitTask({20, 65}, 0, 127).AddRun([]{
      discardBack();
    }),
    TurnToPointSmooth({4, 75}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({4, 75}, 0, 127),

    // Goal 6
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(1).TimeLimit(1500), //Expecting numBallsInRobot == 1
    TimeBasedMoveTask(-70, 400),

    TurnToPointSmooth({60, 70.5}, 0.7, 0.5),
    PurePursuitTask({60, 70.5}, 0, 127).AddRun([]{
      discardBack();
    }),

    // END
    TimeBasedMoveTask(0, 10000000).AddRun([]{
      stopMotors();
    }),
  })
);