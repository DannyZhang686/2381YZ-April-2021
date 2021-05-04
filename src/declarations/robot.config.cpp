
#include "config/robot.config.hpp"

DriveConfig Z_Bot_Drive =  {
    {left_back, {9, 1}},
    {left_front, {5, 1}},
    {right_back, {3, 0}},
    {right_front, {6, 0}},
};

DriveConfig Evan_Bot_Drive = {
    {left_back, {9, 1}},
    {left_front, {5, 1}},
    {right_back, {3, 0}},
    {right_front, {6, 0}},
};


DriveConfig Y_Bot_Drive = {
    {left_back, {11, 0}},
    {left_front, {20, 0}},
    {right_back, {16, 1}},
    {right_front, {17, 1}},
};

DriveConfig L_Bot_Drive = {
    {left_back, {3, 0}},
    {left_front, {11, 1}},
    {right_back, {4, 1}},
    {right_front, {1, 0}},
};
TrackingConfig L_Track_C =
{
    {H, {3,4,1}},
    {V, {1,2,0}},
    {I, {9, 0, false}},
    // true / false is for double (true) or single (false) inertial sensors
};

TrackingConfig Z_Track_C =
{
    {H, {5,6,1}},
    {V, {7,8,1}},
    {I, {4, 11, true}},
};

TrackingConfig E_Track_C =
{
    {H, {3,4,0}},
    {V, {1,2,1}},
    {I, {11, 0, false}},
};


std::complex<double> L_Tracking_Offsets =
{
 2.75,
 5.75
}; // Left to mid, back to mid

std::complex<double> Z_Tracking_Offsets =
{
 2.75,
 5.25
}; // Left to mid, back to mid

DrivePidConfig Z_Bot_Drive_Config =
{
    {left_back, {0.6, 0, 0}},
    {left_front, {0.6, 0, 0}},
    {right_back, {0.6, 0, 0}},
    {right_front, {0.6, 0, 0}},
};
DrivePidConfig Y_Bot_Drive_Config =
{
    {left_back, {0.6, 0, 0}},
    {left_front, {0.6, 0, 0}},
    {right_back, {0.6, 0, 0}},
    {right_front, {0.6, 0, 0 }},
};