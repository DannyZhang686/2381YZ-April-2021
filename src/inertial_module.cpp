#include "inertial_module.hpp"
#include "utilities.h"

using namespace std;
using namespace pros;

Inertial::Inertial(int port1) : mode(A)
{
    imu1 = new Imu(port1);
}
Inertial::Inertial(int port1, int port2) : mode(B)
{
    imu1 = new Imu(port1);
    imu2 = new Imu(port2);
}

const void Inertial::Reset(void)
{ 
    imu1->reset();
    if(mode == B)
    {
        imu2->reset();
    }
}

const double Inertial::Get_Angle(void)
{ 
    if(mode == A) return degToRad(imu1->get_yaw());
    if(mode == B) return degToRad(0.5*(imu1->get_yaw() + imu2->get_yaw()));
}