#ifndef POSITION_TRACKER_HPP
#define POSITION_TRACKER_HPP

#include "api.h"
#include <array>
#include <map>
#include <complex>
#include "inertial_module.hpp"

class Position_Tracker
{
    public:
    enum Tracking_Encoder
    {
        left_ = 0,
        right_,
        back_
    };

    enum Wheel_Side
    {
        Left = 0,
        Right
    };

    static Position_Tracker* instance();
    void Track_Position();

    const std::complex<double> Get_Position() const;

    const std::complex<double> Get_Displacement() const;

    const std::complex<double> Get_Velocity() const;

    // Get the vector pointing in the current direction
    const std::complex<double> Get_Heading_Vec() const;

    const std::complex<double> Get_Wheel_Position(Wheel_Side side) const;
    const double Get_Wheel_Speed(Wheel_Side side) const;




// Actual anglular offset from the start, excluding Set_Position adjustments to angle origin.
    double absolute_angle = 0;

    // Normalized Current - Previous Angle 

    const double Get_Ang_Vel() const;
    const double Get_Angle() const;
    const double Get_Real_Angle() const;

    std::array<double, 3> current_encoder_values = {0, 0, 0}, last_encoder_values = {0, 0, 0}, position_change = {0, 0, 0};

    void Reset_Position(void);
    void Set_Position(std::complex<double> newPosition = 0, double Angle = 0, std::complex<double> previousPosition = 0, double previousAngle = 0);

    void Reset();
    void Create();

    static std::complex<double> wheel_center_offset;
    static std::complex<double> drive_center_offset;


    protected:
    //Vertical & Horizontal Encoder
    void Update_Real_Angle();

    pros::ADIEncoder* v_enc_ = nullptr;
    pros::ADIEncoder* h_enc_ = nullptr;
    Inertial* inertial_ = nullptr;

    double ang_disp = 0, ang_vel = 0, ang_last = 0, ang_origin = 0, pilons_ang_disp = 0;

    std::complex<double> origin = 0;

    std::complex<double> h_disp = 0, v_disp = 0;
    std::complex<double> real_disp = 0, real_vel = 0;

    std::complex<double> v_vel = 0, h_vel = 0;

    unsigned int velLastChecked = 0;
};

extern const double NormalizeAngle(const double angle, const double multiplier = 1);
// Returns angle between [-PI * Multiplier, + PI * Multiplier].
#endif