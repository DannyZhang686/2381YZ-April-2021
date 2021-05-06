#ifndef __TASK_LAMBDAS__H__
#define __TASK_LAMBDAS__H__

#include <string>
#include <math.h>
#include <vector>
#include <functional>


#include "autonomous.h"


namespace TaskLambdas
{

    /**
   * @brief Intake Single Run Lambda - Previously Intake No Shoot
   * @param intakeSpeed Intake Voltage default 200
   *
   */
    extern int runNumber;
    runFn_t PrintLocation(std::string text);


    runFn_t IntakeF(double intakeSpeed = 200);

    /**
  * @brief Lambda Sets Drive To 0 With Both Safe and PID Drive
  **/
    runFn_t SetDrive0() ;

    // Uses `SetDriveSafe(velocity, velocity)` for `time` ms duration, then SetDrive0
    AutoTask TimeBasedMoveTask(double velocity, double time);

}

#endif  //!__TASK_LAMBDAS__H__