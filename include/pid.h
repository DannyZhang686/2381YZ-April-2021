#include "main.h"

#ifndef _PID_H_
#define _PID_H_

//PID struct
typedef struct Pid {
  //Variables used in PID
  double kP, kI, kD, prevError, errorSum;
  //Core function to be called
  double getOutput(double currentValue, double setpoint);
  //Other struct functions
  void reset() {
    kP = kI = kD = prevError = errorSum = 0;
  }
  void resetError() {
    prevError = errorSum = 0;
  }
  //Constructors (default to 0, or take in the appropriate values)
  Pid(): kP(0), kI(0), kD(0) {}
  Pid(double p, double i, double d): kP(p), kI(i), kD(d) {}
} Pid;

//PD struct (generally used with drive)
typedef struct Pd {
  //Core function to be called
  double getOutput(double currentValue, double setpoint);
  //Variables used in the PD controller
  double kP, kD, prevError;
  void reset() {
    kP = kD = prevError = 0;
  }
  void resetError() {
    prevError = 0;
  }
  //Constructors (default to 0, or take in the appropriate values)
  Pd(): kP(0), kD(0) {}
  Pd(double p, double d): kP(p), kD(d) {}
} Pd;

//Extern structs
extern Pd leftStraight, leftTurn, rightStraight, rightTurn;

#endif //_PID_H_
