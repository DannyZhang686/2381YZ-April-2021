#include "main.h"

#ifndef _OPCONTROL_H_
#define _OPCONTROL_H_

//Constants for functions
#define SHOOTER_SPEED 12000
#define INTAKE_SPEED 10000
#define INDEXER_SPEED 10000

//opcontrol function declarations

extern bool shooterOn, intakeOn, indexerOn;

static double shooterSetpoint = 0;
const void controllerShooterSpin(void);
void shooterSpin(void*);

static double intakeSetpoint = 0;
const void controllerIntakeSpin(void);
void intakeSpin(void*);

static double indexerSetpoint = 0;
const void controllerIndexerSpin(void);
void indexerSpin(void*);

void splitArcade(void);

enum DriveMode 
{
    PidMode = 0,
    ManualMode
};

extern DriveMode activeDriveMode;

const void Controller_Set_Drive(double left_x, double left_y, double right_x, double right_y);
const std::array<double, 4> Set_Drive(double lb, double lf, double rb, double rf);
const void Set_Drive_Direct(double lb, double lf, double rb, double rf);

void PID_Drive(void*);

// void Set_Drive(double left_x, double left_y, double right_x, double right_y);

#endif //_OPCONTROL_H_
