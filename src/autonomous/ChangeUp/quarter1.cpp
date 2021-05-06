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
    PurePursuitTask({20, 0}, 0 , 80).AddRun([]{
      s__t(3, "Hello");
    }), //COMMENT TO CHANGE

    SingleRun([]{
        s__t(1, "new task");
    }),
    // TurnToPointSMOOTH({-50, 50}, 100, 0.5),
    // PurePursuitTask({16, 34}, 0 , 80).AddInit(IntakeF(200)).AddKill(PrintLocation("ball on wall")), //COMMENT TO CHANGE

    })
);