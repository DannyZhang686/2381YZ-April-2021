#include "main.h"

#include "api.h"
#include <math.h>
#include "globals.hpp"
#include <list>
#include <functional>
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
    IntakeShootTask(2),
    TimeBasedMoveTask(-60, 400),
/*
    SingleRun([](void) -> void {
      position_tracker->Set_Position({57.6, 14.4}, 2.75762);
    }),
    PurePursuitTask({8.7, 29}, 0, 127).AddInit([]{
      IntakeF(200);
    }),
    TimeBasedMoveTask(0, 200),
    TimeBasedMoveTask(-60, 400),
    TurnToPointSmooth({10, 12}, 0.7, 0.5).AddKill([]{
      stopMotors();
    }),
    PurePursuitTask({10, 12}, 0, 80),

    // Goal 1
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(2), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 400),
*/
    // TurnToPointSmooth({39, 69}, 0.7, 0.5).AddKill([]{
    //   stopMotors();
    // }),
    // PurePursuitTask({39, 69}, 0, 80),
//39, 69

    // END
    TimeBasedMoveTask(0, 10000000),
  })
);
