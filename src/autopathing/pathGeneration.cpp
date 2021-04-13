#include "main.h"
#include "opcontrol.h"
#include "motors.h"
#include "utilities.h"
#include "control/motor_controller.hpp"

#include "autonomous.h"
#include "utilities.h"
#include "pid.h"
#include "autonomous/auto_task.hpp"

#include "control/pid.hpp"
#include "globals.hpp"
#include <math.h> /* pow */
#include <numeric>
#include <complex>
#include <array>
#include <cmath>
#include <vector>

#include "pathing.hpp"



using namespace std;
using namespace pros;
using namespace std::complex_literals;