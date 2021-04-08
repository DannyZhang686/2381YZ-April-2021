#ifndef __ROBOT_CONFIG__H__
#define __ROBOT_CONFIG__H__

#include "config_types.hpp"

static DriveConfig Z_Bot_Drive = {
    {left_back, {6, 1}},
    {left_front, {9, 0}},
    {right_back, {18, 0}},
    {right_front, {19, 1}},
};

static DrivePidConfig Z_Bot_Drive_Config = 
{
    {left_back, {6, 1, 2}},
    {left_front, {9, 0, 2}},
    {right_back, {18, 0, 2}},
    {right_front, {19, 1, 2 }},
};

static DriveConfig Y_Bot_Drive = {
    {left_back, {11, 0}},
    {left_front, {20, 0}},
    {right_back, {16, 1}},
    {right_front, {17, 1}},
};


static DrivePidConfig Y_Bot_Drive_Config = 
{
    {left_back, {0.6, 0, 0}},
    {left_front, {0.6, 0, 0}},
    {right_back, {0.6, 0, 0}},
    {right_front, {0.6, 0, 0 }},
};


#endif  //!__ROBOT_CONFIG__H__