#ifndef __CONFIG_TYPES__H__
#define __CONFIG_TYPES__H__

#include <tuple>
#include <map>

#include "main.h"

enum Motor_Ref
{
    left_back = 0,
    left_front,
    right_back,
    right_front
};

enum Tracking_Ref
{
 H = 0,
 V, 
 I,
 Offsets
};

enum ConfigOptions
{
    Z = 0,
    Y,
    E,
    L
};

typedef std::tuple<int, bool> MotorConfig; // Port, Orientation
typedef std::tuple<int, int, bool> EncoderConfig; // Port, Orientation

typedef std::map<Tracking_Ref, EncoderConfig> TrackingConfig;
typedef std::map<Motor_Ref, MotorConfig> DriveConfig;
typedef std::map<Motor_Ref, std::array<double,3>> DrivePidConfig;

#endif  //!__CONFIG_TYPES__H__