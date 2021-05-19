#include "main.h"
#include "motors.h"
#include "autonomous/position_tracker.hpp"
#include "opcontrol.h"
#include "config/robot.config.hpp"
#include "globals.hpp"

#include <tuple>
#include <map>

//Motor ports

#define LI_PORT 20
#define RI_PORT 10
#define INDEXER_PORT 4
#define SHOOTER_PORT 1
// FIXME - SHOULD BE SHOOTER PORT 1 BUT W/E

//IMU ports
#define L_IMU_PORT 4
#define R_IMU_PORT 11

//Encoder and line sensor ports
//In encoder ports, the "out" port is always
//the port immediately following the "in" port
// #define RIGHT_IN 7
// #define RIGHT_OUT 8
#define BACK_IN 5
#define BACK_OUT 6
#define LEFT_IN 7
#define LEFT_OUT 8

#define TOP_LINE 7
#define MID_LINE 5
#define MID_BOTTOM_LINE 6
#define BOTTOM_LINE 8

//Drive
//Motor reversing accounts for the way the robot is built

using namespace std;
using namespace pros;


// Default Competition Robot Tracking Setup
const complex<double> Position_Tracker::wheel_center_offset =
    {
     2.75,
     5.75
    }; // Left to mid, back to mid
    // Real is distance to vertical track wheel, Imag is distance to horizontal track wheel
const complex<double> Position_Tracker::drive_center_offset = { 0, 6 };

bool shooterOn = true, intakeOn = true, indexerOn = true;

const void InitEncoders(TrackingConfig config)
    {
    leftTracking = new ADIEncoder(get<0>(config[V]), get<1>(config[V]), get<2>(config[V]));
    backTracking = new ADIEncoder(get<0>(config[H]), get<1>(config[H]), get<2>(config[H]));
    inertial = get<2>(config[I]) ? new Inertial(get<0>(config[I]), get<1>(config[I])) : new Inertial(get<0>(config[I]));
    }

const void InitDrive(DriveConfig config)
    {
    leftBack = new pros::Motor(get<0>(config[left_back]), get<1>(config[left_back]));
    leftFront = new pros::Motor(get<0>(config[left_front]), get<1>(config[left_front]));
    rightBack = new pros::Motor(get<0>(config[right_back]), get<1>(config[right_back]));
    rightFront = new pros::Motor(get<0>(config[right_front]), get<1>(config[right_front]));
    }

const void SetTrackingOffsets(complex<double> trackingOffset)
    {
    // Position_Tracker::wheel_center_offset = trackingOffset;
    }

const void InitMotors(ConfigOptions config)
    {
    switch (config)
        {
            case Y:
                InitDrive(Y_Bot_Drive);
                InitMotorControllers(Y_Bot_Drive_Config);
                InitEncoders(L_Track_C);
                break;
            case E:
                InitDrive(Evan_Bot_Drive);
                InitMotorControllers(Z_Bot_Drive_Config);
                // InitEncoders(E_Track_C);
                // SetTrackingOffsets(L_Tracking_Offsets);
                break;
            case L:
                InitDrive(L_Bot_Drive);
                InitMotorControllers(Z_Bot_Drive_Config);
                InitEncoders(L_Track_C);
                SetTrackingOffsets(L_Tracking_Offsets);
                indexerOn = false;
                intakeOn = false;
                shooterOn = false;
                inertial->Reset();
                break;
            default:
                InitDrive(Z_Bot_Drive);
                InitMotorControllers(Z_Bot_Drive_Config);
                InitEncoders(Z_Track_C);
                SetTrackingOffsets(L_Tracking_Offsets);
                inertial->Reset();
                break;
        }
    }

//Intakes
pros::Motor leftIntake(LI_PORT, false);
pros::Motor rightIntake(RI_PORT, true);

//Other
pros::Motor indexer(INDEXER_PORT, true);
pros::Motor shooter(SHOOTER_PORT, false);

//IMUs

//ADI (Encoders and line sensors)
// pros::ADIEncoder rightTracking (RIGHT_IN, RIGHT_OUT, true);

pros::ADIAnalogIn tLineSensor(TOP_LINE);
pros::ADIAnalogIn mLineSensor(MID_LINE);
pros::ADIAnalogIn mbLineSensor(MID_BOTTOM_LINE);
pros::ADIAnalogIn bLineSensor(BOTTOM_LINE);

pros::Controller master(CONTROLLER_MASTER);
