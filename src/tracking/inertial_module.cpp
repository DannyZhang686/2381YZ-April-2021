#include "inertial_module.hpp"
#include "utilities.h"
#include <complex>
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
const bool Inertial::IsCalibrating(void)
{
    return imu1->is_calibrating() || (mode == B && imu2->is_calibrating());
}

const void Inertial::Reset(void)
{ 
    // imu1->
    imu1->reset();
    if(mode == B)
    {
        imu2->reset();
    }
}

const double Inertial::Get_Angle(void)
{ 
    if(mode == A) return degToRad(imu1->get_heading());


    if(mode == B)
    {
        double imu1Angle = degToRad(imu1->get_heading());
        double imu2Angle = degToRad(imu2->get_heading());

        double angleDiff = NormalizeAngle(imu1Angle - imu2Angle, 0.5);
        double actualAngle = NormalizeAngle(imu2Angle + angleDiff/2);

        return actualAngle;
    }
    return 0;
}

const void Inertial::Update_Gyro(void)
{
    gyroAngle +=  imu1->get_gyro_rate().y; 
}
const double Inertial::Get_Gyro(void)
{
    return degToRad(imu1->get_yaw());
}