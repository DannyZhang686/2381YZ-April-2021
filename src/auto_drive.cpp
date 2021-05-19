#include "control/model_predictive_control.hpp"
#include "main.h"
#include "opcontrol.h"
#include "config/profiling.config.hpp"
#include "globals.hpp"
#include "drive.hpp"

using namespace std;

MPC leftWheelControl = ModelPredictiveControl(MOVE_EXPONENT_CONST, MAX_MOVE_SPEED_CONST, 127, 0.5);
MPC rightWheelControl = ModelPredictiveControl(MOVE_EXPONENT_CONST, MAX_MOVE_SPEED_CONST, 127, 0.5);


const array<double, 2> Calc_Drive_MPC(double leftVelTarget, double rightVelTarget, bool print)
    {

    if (inertial->IsCalibrating()) return { 0,0 };
    double leftWheelSpeed = position_tracker->Get_Wheel_Speed(Position_Tracker::Wheel_Side::Left), rightWheelSpeed = position_tracker->Get_Wheel_Speed(Position_Tracker::Wheel_Side::Right);

    double leftPower = leftWheelControl.CalculateVoltage(leftWheelSpeed, leftVelTarget);
    double rightPower = rightWheelControl.CalculateVoltage(rightWheelSpeed, rightVelTarget);
    // s__t(1, "l " + t__s(leftWheelSpeed) + " / " + t__s(leftPower) + " / " + t__s(leftVelTarget));
    // s__t(2, "l " + t__s(rightWheelSpeed) + " / " + t__s(rightPower) + " / " + t__s(rightVelTarget));



    if (print)
        {
        // auto a = iteration9 + "x";
        // std::string hellowtf =
            // t__s(iteration9) + "x" + t__s(leftWheelSpeed - leftPrevState) + "x" + t__s(prevPredictedStateChange) + "x" + t__s(leftPower) + "x" + t__s(leftPrevTarget) + "x" + t__s(leftWheelSpeed) + "\n";
        // printf(hellowtf.c_str());
        // iteration9++;
        }

    return { leftPower, rightPower };
    }