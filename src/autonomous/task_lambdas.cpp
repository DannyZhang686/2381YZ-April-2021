#include <string>
#include <math.h>
#include <vector>
#include <functional>

#include "task_lambdas.hpp"

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
    int runNumber = 0;
    runFn_t PrintLocation(std::string text)
    {
        return [&, text] {
            std::string a = "(" + text + ") RUN: " + t__s(runNumber) + " pos: " + t__s(position_tracker->Get_Position().real()) + ", " + t__s(position_tracker->Get_Position().imag()) + " \n";
            printf(a.c_str());
            runNumber++;
        };
    }


    runFn_t IntakeF(double intakeSpeed)
    {
        return [intakeSpeed] {
            intakeNoShoot(intakeSpeed);
        };
    };

    runFn_t DiscardFront()
    {
        return [] {
            discardFront();
        };
    };

    runFn_t DiscardBack()
    {
        return [] {
            discardBack();
        };
    };

    /**
  * @brief Lambda Sets Drive To 0 With Both Safe and PID Drive
  **/
    runFn_t SetDrive0() {
        return [] {
            Set_Drive_Direct(0, 0, 0, 0);
        };
    };

    // Uses `SetDriveSafe(velocity, velocity)` for `time` ms duration, then SetDrive0
    AutoTask TimeBasedMoveTask(double velocity, double time)
    {
        return Delay(time).AddRun([velocity] { Set_Drive_Direct(velocity, velocity, velocity, velocity); }).AddKill(SetDrive0()).AddKill(PrintLocation(""));
    };



    AutoTask AutoDelay(int time, bool isSync)
    {
        int* currentTime;

        auto init = [&]
        {
            currentTime = new int(0);
        };

        auto run = [&]
        {
            (*currentTime) += DELAY_INTERVAL;
        };

        auto done = [&, time](void)->bool
        {
            return (*currentTime) > time;
        };

        auto kill = [&]
        {
            delete currentTime;
        };
        return AutoTask(run, done, isSync, init, kill);
    };
}

AutoTask AutoTask::TimeLimit(int time)
{

    AutoTask timedTask = TaskLambdas::AutoDelay(time);
    for (const auto& value : this->runList)
    {
        timedTask.AddRun(value);
    };
    for (const auto& value : this->killList)
    {
        timedTask.AddKill(value);
    }
    for (const auto& value : this->initList)
    {
        timedTask.AddInit(value);
    }
    for (const auto& value : this->doneList)
    {
        timedTask.AddDone(value);
    }
    return timedTask;
}
