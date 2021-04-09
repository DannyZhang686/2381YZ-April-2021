#ifndef __INERTIAL_MODULE__H__
#define __INERTIAL_MODULE__H__

#include "main.h"

typedef struct Inertial
{
    enum TrackMode
    {
        A = 0, 
        B
    };
    TrackMode mode;
    
    pros::Imu* imu1 = nullptr;
    pros::Imu* imu2 = nullptr;

    double gyroAngle = 0;


    Inertial(int port1);
    Inertial(int port1, int port2);


    const void Reset(void);
    const bool IsCalibrating(void);
    const double Get_Angle(void);
    const double Get_Gyro(void);
    const void Update_Gyro(void);


} Inertial;
#endif //!__INERTIAL_MODULE__H__