#include <iostream>
#include <cmath>
#include <iomanip>

#include "Eigen/Dense"
#include <tuple>
#include "riccati_solver.h"
#include "opcontrol.h"
#include "motors.h"

#include "matrix.h"


using namespace std;
using namespace Eigen;

double mInverse = 1, rbSquaredOverJ = 1, C1 = 1, C2 = 1, C3 = 1, C4 = 1;


// template <typename T, const int rows, const int cols>
// Matrix<T, rows, cols> MatrixCellExponent(T s)
// {
//     Matrix<T, rows, cols> realizedMatrix();
//     for(int i = 0; i < rows; i ++)
//     {

//     }
// }

// template<typename matrixBase>
// MatrixBase<matrixBase> termMultiply(MatrixBase<matrixBase> lhs, MatrixBase<matrixBase> rhs)
// {
//     MatrixBase<matrixBase> yeetle({});
//     for(int i = 0; i < lhs.rows; i++)
//     {
//         for(int j = 0; j < lhs.cols; j ++)
//         {
//             yeetle(i,j) = lhs(i,j) * rhs(i,j);
//         }
//     }
//     return yeetle;
// }


string toString(double a)
    {
    ostringstream streamObj;
    streamObj << std::fixed;
    streamObj << std::setprecision(2);
    streamObj << a;
    return streamObj.str();
    }

template <const int stateNumber, const int inputNumber, const int parameterNumber>
struct System
    {

    // #define stateNumber 5 // FIXME: For Intellisense static typing only, comment when done
    // #define inputNumber 6 

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


    struct SysModel
        {
        State_O SO;
        Input_O IO;
        SysModel(State_O sO, Input_O iO) : SO(sO), IO(iO)
            {

            };

        SysModel() : SO(State_O::Zero()), IO(Input_O::Zero())
            {

            };
        };

    typedef Matrix<double, parameterNumber, 1> ParameterArray;
    typedef array<SysModel, parameterNumber> ModelArray;

    struct ParameterList
        {
        ParameterArray parameterValues;
        ModelArray modelExponents;

        ModelArray modelPartials;

        SysModel masterMatrix;


        void UpdatePartials()
            {
            for (int i = 0; i < parameterNumber; i++)
                {
                modelPartials[i] = ParameterList::RealizeModel(modelExponents[i], parameterValues(i, 0));
                }
            };



        ParameterList(SysModel masterCoeffs, ParameterArray initialParams, ModelArray systemExponents)
            : masterMatrix(masterCoeffs), parameterValues(initialParams), modelExponents(systemExponents)
            {

            };

        static SysModel RealizeModel(SysModel exponents, double value)
            {
            SysModel realizedValues({});
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


        SysModel CalcModelPartials(int index = -1)
            {
            SysModel total = SysModel(masterMatrix);
            for (int i = 0; i < parameterNumber; i++)
                {
                total.SO = termMultiplyS(total.SO, (i == index) ? modelExponents[i].SO : modelPartials[i].SO);
                total.IO = termMultiplyI(total.IO, (i == index) ? modelExponents[i].IO : modelPartials[i].IO);
                }
            return total;
            }

        };


    ParameterList params;
    Input_t previousInput;
    State_t predictedStateChange, previousState;

    SysModel systemModel;

    Control_O controlOperator;

    double parameterTuningConstant = 0.0005;

    double previousError = 0, errorSum = 0;

    System(ParameterList params_)
        : params(params_), previousInput(Input_t::Zero()), predictedStateChange(State_t::Zero()), previousState(State_t::Zero())
        {
        params.UpdatePartials();
        systemModel = params.CalcModelPartials();
        }


    SysModel TuneModel(State_t newState, Input_t newInput)
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

        systemModel = params.CalcModelPartials();


        predictedStateChange = (systemModel.SO * newState + systemModel.IO * newInput);
        previousState = newState;
        previousInput = newInput;

        return systemModel;
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

// template<>
// Drive::SysModel Drive::TuneModel(State_t newState, Input_t newInput)
//     {

//     };


Drive MakeDriveModel()
    {
    // (A+B), (A-B), C1, C2 In that order
    using Model = Drive::SysModel;
    Drive::ParameterArray initialCoeffList({ 0.2, 0.02, -0.6, 0.2 });
    Drive::ModelArray ModelExponents({ Model(), Model(), Model(), Model() });

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

    Drive::SysModel MasterCoefficients({});

    MasterCoefficients.SO << 1, 1, 1, 1;
    // MasterCoefficients.SO << 1, 0, 0, 1;

    MasterCoefficients.IO << 1, 1, 1, 1;
    // MasterCoefficients.IO << 1, 0, 0, 1;


    Drive::ParameterList parameterList(MasterCoefficients, initialCoeffList, ModelExponents);

    return Drive(parameterList);
    }

Drive DriveModel = MakeDriveModel();


double mod127(double a)
    {
    if (abs(a) < 128)
        {
        return a;
        }
    if (a > 0)
        {
        return 127;
        }
    return -127;
    }

Drive::State_t currentState, previousState = Drive::State_t::Zero();
Drive::Input_t currentInput, previousInput = Drive::Input_t::Zero();

double leftPos = 0, rightPos = 0;

void PID_Drive2(void*)
    {
    double timeSinceLast = 0;
    while (true)
        {
        auto LeftVertical = master.get_analog(ANALOG_LEFT_Y); // -127 <-> 127
        auto RightHorizontal = master.get_analog(ANALOG_RIGHT_X);

        auto leftInput = mod127(LeftVertical + RightHorizontal);
        auto rightInput = mod127(LeftVertical - RightHorizontal);



        auto leftPosNew = (leftBack->get_position() + leftFront->get_position()) / 2;
        auto rightPosNew = (rightBack->get_position() + rightFront->get_position()) / 2;

        auto leftVelocity = (leftBack->get_actual_velocity() + leftFront->get_actual_velocity()) / 2;
        auto rightVelocity = (rightBack->get_actual_velocity() + rightFront->get_actual_velocity()) / 2;


        auto leftSpeed = leftVelocity;
        auto rightSpeed = rightVelocity;

        // auto leftSpeed = leftPosNew - leftPos;
        // auto rightSpeed = rightPosNew - rightPos;

        leftPos = leftPosNew;
        rightPos = rightPosNew;

        leftBack->move(leftInput);
        leftFront->move(leftInput);
        rightBack->move(rightInput);
        rightFront->move(rightInput);

        currentState << leftSpeed, rightSpeed;
        currentInput << leftInput, rightInput;


        // DriveModel.TuneModel(currentState, currentInput);



        // s__t(1, "predicted state: {" + toString(DriveModel.predictedStateChange(0, 0)) + ", " + toString(DriveModel.predictedStateChange(1, 0)) + "}");
        // s__t(2, "real state: {" + toString(currentState(0, 0)) + ", " + toString(currentState(1, 0)) + "}");

        // s__t(3, "vals: {" + toString(DriveModel.params.parameterValues(0,0)) + ", " + toString(DriveModel.params.parameterValues(1,0)) + ", " + toString(DriveModel.params.parameterValues(2,0)) + ", " + toString(DriveModel.params.parameterValues(3,0)) + "}");

        // s__t(4, "er: " + toString(DriveModel.previousError) + " sum: " + toString(DriveModel.errorSum));
        // s__t(5, "s: {" + toString(DriveModel.systemModel.SO(0, 0)) + ", " + toString(DriveModel.systemModel.SO(0, 1)) + ", " + toString(DriveModel.systemModel.SO(1, 0)) + ", " + toString(DriveModel.systemModel.SO(1, 1)) + "}");
        // s__t(6, "i: {" + toString(DriveModel.systemModel.IO(0, 0)) + ", " + toString(DriveModel.systemModel.IO(0, 1)) + ", " + toString(DriveModel.systemModel.IO(1, 0)) + ", " + toString(DriveModel.systemModel.IO(1, 1)) + "}");

        timeSinceLast += 20;

        if (!(currentState == Drive::State_t::Zero() && previousState == Drive::State_t::Zero() && previousInput == Drive::Input_t::Zero()) && (timeSinceLast) > 100)
            {
            std::string a = "Numbers " + t__s(previousState(0,0)) + " "  + t__s(previousState(1,0)) + " " + t__s(previousInput(0,0)) + " " + t__s(previousInput(1,0)) + " " + t__s(currentState(0,0)) + " " + t__s(currentState(1,0)) + "\n";
            printf(a.c_str());
            timeSinceLast = 0;
            }

        previousState = currentState;
        previousInput = currentInput;
        pros::delay(20);

        }
    }

// Drive::TuneModel()

// void InitSomeStuff()
//     {
//     outputOperator <<
//         (1 / m + r * r / J) * C2,
//         (1 / m - r * r / J)* C4,
//         (1 / m - r * r / J)* C2,
//         (1 / m + r * r / J)* C4;
//     };
// void Drive_Do_Stuff(Drive::State_t referenceState)
//     {
//     currentState = getCurrentState();
//     auto refineModel = Drive::TuneModel(currentState, input, currentState, predictedStateChange, input);
//     stateOperator = get<0>(refineModel);
//     outputOperator = get<1>(refineModel);

//     controlOperator = Drive::OptimizeControlLaw(stateOperator, outputOperator, stateCost, inputCost);

//     input = controlOperator * (referenceState - currentState);
//     predictedStateChange = stateOperator * currentState + outputOperator * input;

//     };


// Matrix<double, 2, 3> FUCK;
// Matrix<double, 3, 1> YOU;

// auto a = FUCK * YOU;

// template<int rows, int cols>
// struct matrix
// {
//     // typedef array<double, two> meme;

//     matrix<cols, rows> transpose();
// };


// template<int three, int four>
// auto HIIII()
// {
//     matrix<three, four> kkboi;

//     auto helloThere = kkboi.transpose();
//      return helloThere;
// }

// void doSmthElse()
// {
//     HIIII<3,4>();
// }