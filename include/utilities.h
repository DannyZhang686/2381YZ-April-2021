#include "main.h"
#include "autonomous.h"

#ifndef _UTILITIES_H_
#define _UTILITIES_H_

void setDriveSafe(double, double);
double findDistance(Point, Point);
double findAngle(Point, Point);
double smallestAngle(double, double);
double angleToInches(double);

int sgn(int);
int sgn(float);
int sgn(double);
int sgn(bool);
double encToInches(int);

#endif //_UTILITIES_H_
