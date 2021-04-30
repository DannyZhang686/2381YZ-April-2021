#ifndef __ROBOT_CONFIG__H__
#define __ROBOT_CONFIG__H__

#include "config_types.hpp"

extern DriveConfig Z_Bot_Drive;

extern DriveConfig Evan_Bot_Drive;

// 1 is top roller

extern DriveConfig Z_Bot_Drive;

extern DriveConfig Y_Bot_Drive;
extern DriveConfig L_Bot_Drive;

extern TrackingConfig L_Track_C;
extern TrackingConfig Z_Track_C;

extern std::complex<double> L_Tracking_Offsets;
extern std::complex<double> Z_Tracking_Offsets;
extern TrackingConfig E_Track_C;

extern DrivePidConfig Y_Bot_Drive_Config;
extern DrivePidConfig Z_Bot_Drive_Config;
extern DrivePidConfig L_Bot_Drive_Config;


#endif  //!__ROBOT_CONFIG__H__
