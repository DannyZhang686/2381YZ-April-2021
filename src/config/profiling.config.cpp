#include "config/profiling.config.hpp"
#include "main.h"

// #define ROBOT_Z
// TODO: Comment / uncomment above line to change config.

#ifdef ROBOT_Z
const double MAX_TURN_SPEED_CONST = 0.15875;
const double TURN_EXPONENT_CONST = -0.1135;
const double MAX_TURN_ACCEL_CONST = -TURN_EXPONENT_CONST * MAX_TURN_SPEED_CONST;

const double MAX_MOVE_SPEED_CONST = 1.6; //
const double MOVE_EXPONENT_CONST = -0.105;
const double MAX_MOVE_ACCEL_CONST = -MOVE_EXPONENT_CONST * MAX_MOVE_SPEED_CONST; //
// #undef ROBOT_Z
#endif


#ifndef ROBOT_Z
// ROBOT_L CONFIG
const double MAX_TURN_SPEED_CONST = 12 * M_PI / 180; // 0.191986217719
const double TURN_EXPONENT_CONST = -0.1135;
const double MAX_TURN_ACCEL_CONST = -TURN_EXPONENT_CONST * MAX_TURN_SPEED_CONST; // 0.02377

const double MAX_MOVE_SPEED_CONST = 1.6; //
const double MOVE_EXPONENT_CONST = -0.10;
const double MAX_MOVE_ACCEL_CONST = -MOVE_EXPONENT_CONST * MAX_MOVE_SPEED_CONST; //
#endif

#ifdef ROBOT_Z
#undef ROBOT_Z
#endif