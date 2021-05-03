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

AutoSequence Auton::CUS_Q4 = AutoSequence(
  list<AutoTask>({
   
    })
);