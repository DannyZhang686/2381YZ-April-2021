#include <string>
#include <math.h>
#include <vector>
#include <functional>

#include "task_lambdas.hpp"


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

    /**
  * @brief Lambda Sets Drive To 0 With Both Safe and PID Drive
  **/
    runFn_t SetDrive0() {
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