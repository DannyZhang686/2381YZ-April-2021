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


    Inertial(int port1);
    Inertial(int port1, int port2);


    const void Reset(void);

    const double Get_Angle(void);


} Inertial;
#endif //!__INERTIAL_MODULE__H__