#include <iostream>
#include <cmath>
#include <iomanip>

#include "../include/Eigen/Dense"
#include <tuple>
#include "../include/riccati_solver.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>

using namespace std;
using namespace Eigen;


string toString(double a)
    {
    ostringstream streamObj;
    streamObj << std::fixed;
    streamObj << std::setprecision(2);
    streamObj << a;
    return streamObj.str();
    }


double parameterTuningConstant = 0.01;


template <const int stateNum, const int inputNum, const int parameterNumber>
struct System
    {

#define stateNumber stateNum // FIXME: For Intellisense static typing only, comment when done
#define inputNumber inputNum 

    static const int paramN = parameterNumber;
    static const int stateN = stateNumber;
    static const int inputN = inputNumber;

    typedef Matrix<double, stateNumber, 1> State_t;
    typedef Matrix<double, inputNumber, 1> Input_t;

    typedef Matrix<double, stateNumber, stateNumber> State_O;   // state = SO * state
    typedef Matrix<double, stateNumber, inputNumber> Input_O;  // state = IO * input
    typedef Matrix<double, inputNumber, stateNumber> Control_O; // input = CO * state;
    typedef Matrix<double, inputNumber, inputNumber> Output_O;   // input = OO * input;

    static State_O termMultiplyS(State_O lhs, State_O rhs)
        {
        State_O yeetle = State_O::Zero();
        for (int i = 0; i < lhs.rows(); i++)
            {
            for (int j = 0; j < lhs.cols(); j++)
                {
                yeetle(i, j) = lhs(i, j) * rhs(i, j);
                }
            }
        return yeetle;
        }
    static Input_O termMultiplyI(Input_O lhs, Input_O rhs)
        {
        Input_O yeetle = Input_O::Zero();
        for (int i = 0; i < lhs.rows(); i++)
            {
            for (int j = 0; j < lhs.cols(); j++)
                {
                yeetle(i, j) = lhs(i, j) * rhs(i, j);
                }
            }
        return yeetle;
        }


    struct SystemOperators
        {
        State_O SO;
        Input_O IO;
        SystemOperators(State_O sO, Input_O iO) : SO(sO), IO(iO)
            {

            };

        SystemOperators() : SO(State_O::Zero()), IO(Input_O::Zero())
            {

            };

        State_t predictedStateChange(State_t state, Input_t input)
            {
            return (SO * state + IO * input);
            }
        };

    typedef Matrix<double, parameterNumber, 1> ParameterArray;
    typedef array<SystemOperators, parameterNumber> ModelArray;

    struct SystemModel
        {
        ParameterArray parameterValues;
        ModelArray modelExponents;

        ModelArray modelPartials;

        SystemOperators masterMatrix;

        SystemOperators systemOperators;


        void UpdatePartials()
            {
            for (int i = 0; i < parameterNumber; i++)
                {
                modelPartials[i] = SystemModel::RealizeModel(modelExponents[i], parameterValues(i, 0));
                }
            systemOperators = CalcModelPartials();
            };



        SystemModel(SystemOperators masterCoeffs, ParameterArray initialParams, ModelArray systemExponents)
            : masterMatrix(masterCoeffs), parameterValues(initialParams), modelExponents(systemExponents)
            {

            };

        static SystemOperators RealizeModel(SystemOperators exponents, double value)
            {
            SystemOperators realizedValues({});
            for (int i = 0; i < stateNumber; i++)
                {
                for (int j = 0; j < stateNumber; j++)
                    {
                    // realizedValues(i,j) = pow(value, partial(i,j)) but ternaried so not using pow every time for simple operations.
                    realizedValues.SO(i, j) = exponents.SO(i, j) == 0 ? 1 : value;
                    }
                for (int j = 0; j < inputNumber; j++)
                    {
                    // realizedValues(i,j) = pow(value, partial(i,j)) but ternaried so not using pow every time for simple operations.
                    realizedValues.IO(i, j) = exponents.IO(i, j) == 0 ? 1 : value;
                    }
                }
            return realizedValues;
            };


        SystemOperators CalcModelPartials(int index = -1)
            {
            SystemOperators total = SystemOperators(masterMatrix);
            for (int i = 0; i < parameterNumber; i++)
                {
                total.SO = termMultiplyS(total.SO, (i == index) ? modelExponents[i].SO : modelPartials[i].SO);
                total.IO = termMultiplyI(total.IO, (i == index) ? modelExponents[i].IO : modelPartials[i].IO);
                }
            return total;
            }

        };


    SystemModel params;
    Input_t previousInput;
    State_t predictedStateChange, previousState;

    SystemOperators getSystemOperators()
        {
        return params.systemOperators;
        };

    Control_O controlOperator;


    double previousError = 0, errorSum = 0;

    System(SystemModel params_)
        : params(params_), previousInput(Input_t::Zero()), predictedStateChange(State_t::Zero()), previousState(State_t::Zero())
        {
        params.UpdatePartials();
        }

    struct StateInputData
        {
        State_t newState;
        State_t previousState;
        Input_t previousInput;
        StateInputData(State_t oldState, Input_t oldInput, State_t newState)
            : newState(newState), previousState(oldState), previousInput(oldInput)
            {

            }
        };

    static ParameterArray TuneModelStatic(vector<StateInputData> stateInputData, SystemModel params);


    void TuneModel(State_t newState, Input_t newInput)
        {

        auto actualStateChange = newState - previousState;

        previousError = (predictedStateChange - actualStateChange).transpose() * (predictedStateChange - actualStateChange);
        errorSum += previousError;
        ParameterArray gradientVector = ParameterArray::Zero();
        for (int i = 0; i < parameterNumber; i++)
            {
            auto partial = params.CalcModelPartials(i);
            double partialGradient = (predictedStateChange - actualStateChange).transpose() * (partial.SO * previousState + partial.IO * previousInput);
            gradientVector(i, 0) = -partialGradient;
            }



        // gradientVector = gradientVector.normalized() * previousError;
        params.parameterValues += gradientVector * parameterTuningConstant;
        params.UpdatePartials();



        predictedStateChange = params.predictedStateChange(newState, newInput);
        previousState = newState;
        previousInput = newInput;
        };


    Control_O OptimizeControlLaw(State_O A, Input_O B, State_O Q, Output_O R)
        {
        MatrixXd P;
        bool solved = solveRiccatiIterationD(A, B, Q, R, P);

        Control_O K = (R + B.transpose() * P * B).transpose() * B.transpose() * P * A;
        return K;
        };
    };

typedef System<2, 2, 4> Drive;

template<>

Drive::ParameterArray Drive::TuneModelStatic(vector<StateInputData> stateInputData, SystemModel currentModel) {
    ParameterArray gradientVector = ParameterArray::Zero();
    double prevErrorSum = 0;
    double averageChange = 0;

    double errorPercentAverage = 0;
    double updatedErrorSum = 0;

    size_t numTerms = stateInputData.size();
    State_t averageErrorVec = State_t::Zero();
    for (StateInputData stateInput : stateInputData)
        {
        auto newState = stateInput.newState;
        auto previousState = stateInput.previousState;
        auto previousInput = stateInput.previousInput;
        auto stateChange = newState - previousState;
        auto prevPredicted = currentModel.systemOperators.predictedStateChange(previousState, previousInput);

        averageChange += (stateChange).norm();


        double prevError = (prevPredicted - stateChange).norm();

        // errorPercentAverage += prevError / (stateChange).squaredNorm();
        prevErrorSum += prevError / stateInputData.size();
        averageErrorVec += (prevPredicted - stateChange) / stateInputData.size();

        for (int i = 0; i < Drive::paramN; i++)
            {
            auto partial = currentModel.CalcModelPartials(i);
            double partialGradient = (prevPredicted - stateChange).transpose() * (partial.predictedStateChange(previousState, previousInput));
            gradientVector(i, 0) -= partialGradient;
            }
        }


    averageChange = averageChange / stateInputData.size();
    // cout << "averageChange: " << averageChange << ", error : " << prevErrorSum << " ";
    // cout << "errorVX: " << averageErrorVec(0,0) << ", errorVY: " << averageErrorVec(1,0)  << " ";

    gradientVector = gradientVector * parameterTuningConstant / numTerms ;
    currentModel.parameterValues += gradientVector;
    currentModel.UpdatePartials();


    for (StateInputData stateInput : stateInputData)
        {
        auto newState = stateInput.newState;
        auto previousState = stateInput.previousState;
        auto previousInput = stateInput.previousInput;
        auto stateChange = newState - previousState;
        auto prevPredicted = currentModel.systemOperators.predictedStateChange(previousState, previousInput);

        double prevError = (prevPredicted - stateChange).norm();
        updatedErrorSum += prevError;
        };

    cout << "ac " << averageChange << " " << updatedErrorSum / numTerms << " " << prevErrorSum << " tuningCons " << parameterTuningConstant << " ";

    if (updatedErrorSum > prevErrorSum * numTerms)
        {
        currentModel.parameterValues -= gradientVector;
        currentModel.UpdatePartials();
        parameterTuningConstant = 0.8 * parameterTuningConstant;
        return ParameterArray::Zero();
        }
    return gradientVector;
    };



Drive MakeDriveModel()
    {
    // (A+B), (A-B), C1, C2 In that order
    using Model = Drive::SystemOperators;
    // Drive::ParameterArray initialCoeffList({ 0.181675, 0.0173574, -0.863994, 0.346628 });
    // Drive::ParameterArray initialCoeffList({ 0.80851, -0.00205899, -0.268259, 0.371191 });

    
    // Matrix<double, 4, 1> ree({0,0,0,0});
    Drive::ParameterArray initialCoeffList;
    initialCoeffList << 0.326579, -0.069722, -0.421501, 0.541926;
    // initialCoeffList << 0.4, 0.0, -0.4, 0.5;



    // 0.258217, 0.0245166, -0.606826, 0.243945
    Drive::ModelArray ModelExponents({ });

    // For (A + B):
    // {1, 0} * x + {1, 0} * u
    // {0, 1}       {0, 1}

    ModelExponents[0].SO << 1, 0, 0, 1;
    ModelExponents[0].IO << 1, 0, 0, 1;
    // ModelExponents[0] = Model();

    // For (A - B):
    // {0, 1} * x + {0, 1} * u
    // {1, 0}       {1, 0}
    ModelExponents[1].SO << 0, 1, 1, 0;
    ModelExponents[1].IO << 0, 1, 1, 0;
    // ModelExponents[1] = Model();


    // For C1:
    // {1, 1} * x + {0, 0} * u
    // {1, 1}       {0, 0}
    ModelExponents[2].SO << 1, 1, 1, 1;
    ModelExponents[2].IO << 0, 0, 0, 0;

    // For C2:
    // {0, 0} * x + {1, 1} * u
    // {0, 0}       {1, 1}
    ModelExponents[3].SO << 0, 0, 0, 0;
    ModelExponents[3].IO << 1, 1, 1, 1;

    
    // For C2:
    // {0, 0} * x + {1, 1} * u
    // {0, 0}       {1, 1}
    // ModelExponents[4].SO << 1, 0, 0, 0;
    // ModelExponents[4].IO << 0, 0, 0, 0;

    Drive::SystemOperators MasterCoefficients({});

    MasterCoefficients.SO << 1, 1, 1, 1;
    // MasterCoefficients.SO << 1, 0, 0, 1;

    MasterCoefficients.IO << 1, 1, 1, 1;
    // MasterCoefficients.IO << 1, 0, 0, 1;


    Drive::SystemModel SystemModel(MasterCoefficients, initialCoeffList, ModelExponents);

    return Drive(SystemModel);
    }

Drive DriveModel = MakeDriveModel();

vector<Drive::StateInputData> stateInputData{};


int main()
    {
    string myText;
    ifstream MyReadFile("stateNumbers.txt");
    while (getline(MyReadFile, myText))
        {
        istringstream iss(myText);
        vector<string> tokens{ istream_iterator<string>{iss},
                              istream_iterator<string>{} };
        stateInputData.push_back(Drive::StateInputData({ stod(tokens[1]),stod(tokens[2]) }, { stod(tokens[3]),stod(tokens[4]) }, { stod(tokens[5]),stod(tokens[6]) }));
        // cout << stateInputData[0].newState << endl;
        };

    int i = 0;
    while (parameterTuningConstant > 0.0005 && i < 250)
        {
        auto gradient = Drive::TuneModelStatic(stateInputData, DriveModel.params);
        DriveModel.params.parameterValues += gradient;
        DriveModel.params.UpdatePartials();
        gradient = gradient / parameterTuningConstant;

        // cout << "Run: " << i << ", error: " << gradient.norm() << " current: " << stateInputData[0].newState(0, 0) << " params: {" << DriveModel.params.parameterValues(0, 0) << ", " << DriveModel.params.parameterValues(1, 0) << ", " << DriveModel.params.parameterValues(2, 0) << ", " << DriveModel.params.parameterValues(3, 0) << "}" << endl;
        cout << "Run: " << i  << " params: {" << DriveModel.params.parameterValues(0, 0) << ", " << DriveModel.params.parameterValues(1, 0) << ", " << DriveModel.params.parameterValues(2, 0) << ", " << DriveModel.params.parameterValues(3, 0)  << "}" << " params: {" << gradient(0, 0) << ", " << gradient(1, 0) << ", " << gradient(2, 0) << ", " << gradient(3, 0) << "}" << endl;
        i++;
        }
    for (int i = 0; i < 1; i++)
        {
        auto current = stateInputData[i];
        // current.
        auto predicted = DriveModel.params.systemOperators.predictedStateChange(current.previousState, current.previousInput);
        cout << "predicted: {" << predicted(0, 0) << ", " << predicted(1, 0) << "} actual: {" << current.newState(0, 0) - current.previousState(0, 0) << ", " << current.newState(1, 0) - current.previousState(1, 0) << "}" <<
            "state: {" << current.previousState(0, 0) << ", " << current.previousState(1, 0) << "} input: {" << current.previousInput(0, 0) << ", " << current.previousInput(1, 0) << "}" << endl;
        }
    return 0;
    };
