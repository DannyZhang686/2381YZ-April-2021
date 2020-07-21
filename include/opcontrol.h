#include "main.h"

#ifndef _OPCONTROL_H_
#define _OPCONTROL_H_

//Constants for functions
#define SHOOTER_SPEED 12000
#define INTAKE_SPEED 10000
#define INDEXER_SPEED 10000

//opcontrol function declarations
void splitArcade(void*);
void shooterSpin(void*);
void intakeSpin(void*);
void indexerSpin(void*);

#endif //_OPCONTROL_H_
