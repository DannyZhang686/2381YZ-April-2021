#include "main.h"
#include "autonomous.h"

#ifndef _UTILITIES_H_
#define _UTILITIES_H_

int sgn(int);
int sgn(float);
int sgn(double);
int sgn(bool);
double encToInches(int);
double rotatePi(double);
double degToRad(double);
double radToDeg(double);

const double getSignOf(const double);

const double powCalc(const double x, const double power);
const double minMaxMod(const double k, const double max, const double min);
const double minMaxMod(const double k, const double max);




#endif //_UTILITIES_H_
