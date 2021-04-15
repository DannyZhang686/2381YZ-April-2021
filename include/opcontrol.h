#include "main.h"

#ifndef _OPCONTROL_H_
#define _OPCONTROL_H_

//Constants for functions
#define SHOOTER_SPEED 12000
#define INTAKE_SPEED 10000
#define INDEXER_SPEED 10000

//opcontrol function declarations

static bool shooterOn = true, intakeOn = true, indexerOn = true;

void shooterSpin(void*);
void intakeSpin(void*);

static double indexerSetpoint = 0;
void controllerIndexerSpin();
void indexerSpin(void*);

void splitArcade(void);

const void Controller_Set_Drive(double left_x, double left_y, double right_x, double right_y);
const void Set_Drive(double lb, double lf, double rb, double rf);
void PID_Drive(void*);

// void Set_Drive(double left_x, double left_y, double right_x, double right_y);

#endif //_OPCONTROL_H_
