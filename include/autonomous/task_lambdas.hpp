#ifndef __TASK_LAMBDAS__H__
#define __TASK_LAMBDAS__H__

#include <string>
#include <math.h>
#include <vector>
#include <functional>


#include "main.h"
#include "api.h"
#include "globals.hpp"
#include "autonomous.h"
#include "autonomous/functional_tasks.hpp"
#include "opcontrol.h"


namespace TaskLambdas
{

    /**
   * @brief Intake Single Run Lambda - Previously Intake No Shoot
   * @param intakeSpeed Intake Voltage default 200
   *
   */
    extern int runNumber;
    auto PrintLocation(std::string text)
    {
        return [&, text] {
            std::string a = "(" + text + ") RUN: " + t__s(runNumber) + " pos: " + t__s(position_tracker->Get_Position().real()) + ", " + t__s(position_tracker->Get_Position().imag()) + " \n";
            printf(a.c_str());
            runNumber++;
        };
    }


    auto IntakeF(double intakeSpeed = 200)
    {
        return [intakeSpeed] {
            intakeNoShoot(intakeSpeed);
        };
    };

    /**
  * @brief Lambda Sets Drive To 0 With Both Safe and PID Drive
  **/
    auto SetDrive0() {
        return [] {
            setDriveSafe(0, 0);
            Set_Drive(0, 0, 0, 0);
        };
    };

    // Uses `SetDriveSafe(velocity, velocity)` for `time` ms duration, then SetDrive0
    AutoTask TimeBasedMoveTask(double velocity, double time)
    {
        return Delay(time).AddRun([velocity] { setDriveSafe(velocity, velocity); }).AddKill(SetDrive0()).AddKill(PrintLocation(""));
    };

}

#endif  //!__TASK_LAMBDAS__H__