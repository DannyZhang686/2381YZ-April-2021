#include "main.h"
#include "motors.h"
//master
#include "globals.hpp"
#include "control/model_predictive_control.hpp"
#include "utilities.h"
#include "control/state_space_methods.hpp"
#include "control/model_predictive_control.hpp"

void MPC::UpdateModel(double updatedState)
    {

    double predictedStateChange = GetPredictedStateChange(previousState, previousInput);
    maxStateCorrection += Ki * (updatedState - previousState - predictedStateChange) * (-modelExponent) * previousInput / maxInput;
    }

const double MPC::GetPredictedStateChange(double state, double input) const
    {
    return modelExponent * (state - GetCalcMaxState() * input / maxInput);
    }

const double MPC::CalcNewInput(double currentState, double referenceState) const
    {
    double accelTarget = referenceState - currentState;

    double maxAccelBuffer = 3 * abs(modelExponent * GetCalcMaxState() * maxAccelPercent);

    if (abs(accelTarget) > maxAccelBuffer)
        {
        accelTarget = abs(modelExponent * GetCalcMaxState() * maxAccelPercent) * getSignOf(accelTarget);
        }
    else {
        accelTarget = (pow(accelTarget / maxAccelBuffer, 2)) * accelTarget;
        }
    return (maxInput * calcVoltageSetpoint(accelTarget, modelExponent, currentState, GetCalcMaxState()));
    }


double filterWeight = 1;

const double MPC::CalculateVoltage(double currentState, double referenceInput)
    {

    double referenceState = GetCalcMaxState() * minMaxMod(referenceInput / maxInput, 1);
    uint32_t currentTime = pros::millis();
    double predictedPrevState = previousState + GetPredictedStateChange(previousState, previousInput);
    double filteredCurrentState = currentState * filterWeight + predictedPrevState * (1 - filterWeight);


    if (abs(currentTime - lastUpdated) <= DELAY_INTERVAL)
        {
        UpdateModel(currentState);
        }
    else
        {
        filteredCurrentState = currentState;
        }
    double newInput = (CalcNewInput(filteredCurrentState, referenceState));

    lastUpdated = currentTime;
    previousInput = newInput;
    previousState = filteredCurrentState;
    return newInput;
    }


