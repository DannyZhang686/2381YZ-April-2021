#include "main.h"

#ifndef _PROS_OPCONTROL_H_
#define _PROS_OPCONTROL_H_

//constants for functions
#define SHOOTER_SPEED 12000
#define INTAKE_SPEED 10000
#define INDEXER_SPEED 10000

//opcontrol function declarations
void splitArcade(void*);
void shooterSpin(void*);
void intakeSpin(void*);
void indexerSpin(void*);

#endif
