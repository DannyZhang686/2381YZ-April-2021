#include "main.h"

#ifndef _OPCONTROL_H_
#define _OPCONTROL_H_

//Constants for functions
#define SHOOTER_SPEED 12000
#define INTAKE_SPEED 10000
#define INDEXER_SPEED 10000

//opcontrol function declarations
void shooterSpin(void*);
void intakeSpin(void*);
void indexerSpin(void*);

void splitArcade(void);
const void Set_Drive(double left_x, double left_y, double right_x, double right_y);
void PID_Drive(void*);

// void Set_Drive(double left_x, double left_y, double right_x, double right_y);

#endif //_OPCONTROL_H_
