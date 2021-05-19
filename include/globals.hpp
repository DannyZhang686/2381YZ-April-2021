#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "autonomous/auton_control.hpp"
#include "master_controller.hpp"
#include "autonomous/position_tracker.hpp"

#define DELAY_INTERVAL 20
extern bool STOP;
const void stop(void);

extern double autonomous_increment;

extern void driver();

extern bool Competition_Env;
extern std::uint32_t now;
extern MasterController* master_control;
extern AutonControl& auton_control;

extern Position_Tracker* position_tracker;
extern Inertial* inertial;

#endif

