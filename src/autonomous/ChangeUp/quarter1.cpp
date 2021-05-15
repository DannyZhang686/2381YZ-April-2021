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
      position_tracker->Set_Position({57.6, 14.4}, M_PI - 0.384);
    }),
    // IntakeF(200);
    Delay(200),
    PurePursuitTask({9.5, 23.5}, 0, 80),
    // TimeBasedMoveTask(0, 200),
    // TimeBasedMoveTask(-70, 650),
    // TurnToPointSmooth({12, 12}, 100, 0.5).AddKill([]{
    //   stopMotors();
    // }),
    // PurePursuitTask({16, 34}, 0 , 80).AddInit(IntakeF(200)).AddKill(PrintLocation("ball on wall")), //COMMENT TO CHANGE
    Delay(10000000),
  })
);
