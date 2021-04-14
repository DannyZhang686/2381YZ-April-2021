#ifndef POSITION_TRACKER_HPP
#define POSITION_TRACKER_HPP

#include "api.h"
#include <array>
#include <map>
#include <complex>
#include "inertial_module.hpp"

class Position_Tracker {
    public:
    enum Tracking_Encoder {
        left_ = 0,
        right_, 
        back_
    };

    enum Vector_Component {
        XComp = 0,
        YComp,
        WComp,
    };


    static Position_Tracker* instance();
    void Track_Position();
    void Track_Position_Pilons();


    std::complex<double> Get_Position();
    std::complex<double> Get_Position_N();

    std::complex<double> Get_Displacement();

    std::complex<double> Get_Velocity();
    double angle_ = 0;

    double Get_Ang_Vel();
    double Get_Angle();
    double Get_Real_Angle();


    const void Reset_Position(void);
    const void Set_Position(std::complex<double> newPosition = 0, double Angle = 0, std::complex<double> previousPosition = 0, double previousAngle = 0);
    void Reset();

    void Create();
    static const std::complex<double> wheel_center_offset;

    protected:

//Vertical & Horizontal Encoder

    pros::ADIEncoder *v_enc_ = nullptr;
    pros::ADIEncoder * h_enc_ = nullptr;
    Inertial* inertial_ = nullptr;


    double ang_disp = 0, ang_vel = 0, ang_last = 0, ang_origin = 0, pilons_ang_disp = 0;

    std::complex<double> origin = 0;

    std::complex<double> h_disp = 0, v_disp = 0;
    std::complex<double> pilons_disp = 0;


    std::complex<double> v_vel = 0, h_vel = 0;

    std::complex<double> v_vel_n = 0, h_vel_n = 0, h_disp_n = 0, v_disp_n = 0;


    std::array<double,3> current_encoder_values = {0,0,0}, last_encoder_values = {0,0,0}, position_change = {0,0,0};
    unsigned int velLastChecked = 0;
};

double NormalizeAngle(double angle, int multiplier = 1);

#endif