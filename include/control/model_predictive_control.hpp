#ifndef __MODEL_PREDICTIVE_CONTROL__H__
#define __MODEL_PREDICTIVE_CONTROL__H__

struct ModelPredictiveControl
	{

	const double modelExponent, maxInput, initialMaxState, maxAccelPercent, Ki;

	double previousState = 0, maxStateCorrection = 0, previousInput = 0;
	long long lastUpdated = 0;



	const double GetCalcMaxState() const
		{
		return initialMaxState + maxStateCorrection;
		}

	void UpdateModel(double updatedState);

    const double GetPredictedStateChange(double currentState, double currentInput) const;
	const double CalcNewInput(double currentState, double referenceState) const;

	const double CalculateVoltage(double currentState, double referenceInput);
	const void Set_Internal_State(double previousState_, double previousInput_)
	{
		previousState = previousState_;
		previousInput = previousInput_;
	};


	ModelPredictiveControl(double modelExp, double maxState, double maxInput = 127, double maxAccelPercent = 0.5, double Ki = 0.01) 
		: modelExponent(modelExp), initialMaxState(maxState), maxInput(maxInput), maxAccelPercent(maxAccelPercent), Ki(Ki)
	{

	}

    // ModelPredictiveControl(ModelPredictiveControl&);

	};

typedef ModelPredictiveControl MPC;
#endif  //!__MODEL_PREDICTIVE_CONTROL__H__