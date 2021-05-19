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
    SingleRun([](void) -> void {
      position_tracker->Set_Position({86.4, 14.4}, 0.383972);
    }),
    PurePursuitTask({131.5, 34}, 0, 127).AddRun(IntakeF(200)),
    TimeBasedMoveTask(0, 150),
    TimeBasedMoveTask(-60, 500),
    TurnToPointSmooth({129, 15}, 0.7, 0.5).AddKill([]{ //119, 10
      stopMotors();
    }),
    PurePursuitTask({129, 15}, 0, 127),

    // Goal 1
    TimeBasedMoveTask(70, 300),
    // Delay(1000),
    IntakeShootTask(2).TimeLimit(1000), //Expecting numBallsInRobot == 2
    TimeBasedMoveTask(-70, 400),

    TurnToPointSmooth({101, 72}, 0.7, 0.5),
    PurePursuitTask({101, 72}, 0, 127).AddRun([]{
      discardBack();
    }),
    TurnToPointSmooth({129, 72}, 0.7, 0.5).AddRun(IntakeF(200)),
    PurePursuitTask({129, 72}, 0, 127),

    // END
    TimeBasedMoveTask(0, 10000000),
  })
);
