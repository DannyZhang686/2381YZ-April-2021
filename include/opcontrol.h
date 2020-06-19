#include "main.h"

#ifndef _PROS_OPCONTROL_H_
#define _PROS_OPCONTROL_H_

//constants for functions
#define FLYWHEEL_SPEED 300
#define INTAKE_SPEED 150
#define INDEXER_SPEED 200

//opcontrol function declarations
void splitArcade(void*);
void flywheelSpin(void*);
void intakeSpin(void*);
void indexerSpin(void*);

#endif
