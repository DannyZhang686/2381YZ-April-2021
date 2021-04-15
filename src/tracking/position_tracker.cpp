#include "autonomous/position_tracker.hpp"
#include "globals.hpp"
#include "motors.h"
#include "utilities.h"

#include "api.h"
#include <array>
#include <complex>
#include <cmath>

using namespace pros;
using namespace std;
using namespace std::complex_literals;


double NormalizeAngle(double angle, int multiplier)
{
    return remainder(angle, 2 * M_PI * multiplier);
}

Position_Tracker *Position_Tracker::instance()
{
    static Position_Tracker instance;
    return &instance;
}

void Position_Tracker::Create()
{

    inertial_ = inertial;
    v_enc_ = leftTracking;
    h_enc_ = backTracking;

    Reset();
    // Maybe take an input or global on initial position;
    Set_Position(0, 0);
    // Maybe take an input or global on initial position;
    // Set_Position(0, 0);
}

void Position_Tracker::Reset()
{
    inertial_->Reset();
    v_enc_->reset();
    h_enc_->reset();
    current_encoder_values = last_encoder_values = position_change = {0, 0, 0};
    ang_disp = ang_last = ang_vel = ang_origin = 0;
    h_disp = v_disp = h_vel = v_vel = origin = v_disp_n = h_disp_n = 0;
}

const void Position_Tracker::Set_Position(complex<double> position_, double angle_, complex<double> oldPos, double oldAngle)
{
    origin = position_ - oldPos + origin;

    ang_origin = angle_ - oldAngle + ang_origin;
    ang_last = Get_Angle();
}

int calibrationStart = 0;

double Position_Tracker::Get_Real_Angle()
{
    if (inertial_->IsCalibrating())
        return 0;

    angle_ = NormalizeAngle(-inertial_->Get_Angle());
    return angle_;
}
double Position_Tracker::Get_Angle()
{
    return NormalizeAngle(angle_ + ang_origin);
}

double Position_Tracker::Get_Ang_Vel()
{
    return ang_vel;
}

void Position_Tracker::Track_Position_Pilons()
{
}

complex<double> pilonsVel = 0;

void Position_Tracker::Track_Position()
{
    if (inertial_->IsCalibrating())
    {
        lcd::set_text(2, "IMU CALIBRATING");
        return;
    }

    // Current Angle, Angular Dispertion. This is normalized to between (-M_PI, M_PI).
    this->Get_Real_Angle();
    ang_disp = this->Get_Angle();

    // Angular Velocity
    ang_vel = fmod(ang_disp - ang_last, 2 * M_PI);
    auto normalized_ang_vel = NormalizeAngle(ang_disp - ang_last);

    current_encoder_values[right_] = v_enc_->get_value();
    current_encoder_values[back_] = h_enc_->get_value();
    lcd::set_text(2, "VERT: " + to_string(current_encoder_values[right_]) + " Horz: " + to_string(current_encoder_values[back_]));

    // Position change is the swept angle multiplied by the radius. Radius = 1/2 Diameter, so it is \Delta_Angle * M_PI/180 *Diameter/2.
    position_change[right_] = (current_encoder_values[right_] - last_encoder_values[right_]) * PI * TRACKING_WHEEL_DIAMETER / 360;
    position_change[back_] = (current_encoder_values[back_] - last_encoder_values[back_]) * PI * TRACKING_WHEEL_DIAMETER / 360;

    // Current Orientation - Angular Change/2 to get average between current and last angle measured.
    // Angular velocity is normalized to (-M_PI, M_PI) so that the change in angle always calculates as the closest side.
    // Otherwise the vector will be pointing in the opposite direction.

    v_vel = (position_change[right_]) * exp<double>(1i * (ang_disp - ang_vel * 1 / 2));
    h_vel = (position_change[back_]) * exp<double>(1i * (ang_disp - M_PI / 2 - ang_vel * 1 / 2));

    // v_vel_n = (position_change[right_]) * exp<double>(1i * (ang_disp - normalized_ang_vel * 1/2));
    // h_vel_n = (position_change[back_]) * exp<double>(1i * (ang_disp - M_PI/2 - normalized_ang_vel * 1/2));

    // v_disp_n += v_vel_n;
    // h_disp_n += h_vel_n;

    h_disp += h_vel;
    v_disp += v_vel;
    ang_last = ang_disp;
    last_encoder_values = current_encoder_values;
    s__t(1, t__s(position_tracker->Get_Position().real()) + " " + t__s(position_tracker->Get_Position().imag()) + " " + t__s(position_tracker->Get_Angle()));

    //
    // lcd::set_text(1, "VEL: ("  + to_string((int)round( 100* abs(Get_Velocity()))) + ", " + to_string((int)round( 100* abs(Get_Ang_Vel()))) + "), " + to_string(round(ang_disp * 180 / M_PI)) +" deg");
    // lcd::set_text(2, "N_POS : ("  + to_string((int)round( 100* Get_Position_N().real())) + ", " + to_string((int)round( 100* Get_Position_N().imag())) + "), " + to_string(round(ang_disp * 180 / M_PI)) +" deg");

    double lRadius;                                //The radius of the circle calculated to the left tracking wheel
    double bRadius;                                //rRadius calculated for the back tracking wheel (used as an adjustment for when the robot turns or is pushed sideways)
    double dist;                                   //The distance travelled by the tracking center
    double dist2;                                  //dist calculated using the back tracking wheel
    double sinAngle = sin(normalized_ang_vel); //The sine of the angle travelled (used to avoid multiple redundant calculations)

    if (fabs(normalized_ang_vel) > 0)
    {
        lRadius = position_change[right_] / (normalized_ang_vel); //Calculate the radius
        // dist = (rRadius + R_TO_MID) * normalized_ang_vel; //Calculate the distance (from the center of the robot) using simple trigonometry
        dist = (lRadius + wheel_center_offset.real()) * sinAngle;    //Calculate the distance (from the center of the robot) using simple trigonometry
        bRadius = position_change[back_] / (normalized_ang_vel); //Repeat the previous lines using the back tracking wheel (for horizontal error)
        dist2 = (bRadius - wheel_center_offset.imag()) * sinAngle;
    }
    else
    {
        //Robot went straight or didn't move; note that this happens when normalized_ang_vel == 0
        //Values for distance travelled can be set directly to encoder values, as there is no arc
        dist = position_change[right_];
        dist2 = position_change[back_];
    }
    //Update the angle value

    // pilonsVel = dist * exp<double>(1i *(ang_disp) ) + dist2 * exp<double>(1i* -(M_PI/2 + ang_disp)); //Original
    pilonsVel = dist * exp<double>(1i * (ang_disp)) + dist2 * exp<double>(1i * -(M_PI / 2 - ang_disp));
    pilons_disp += pilonsVel;

    // lcd::set_text(2, "N_POS : ("  + to_string((int)round( 100* Get_Position_N().real())) + ", " + to_string((int)round( 100* Get_Position_N().imag())) + "), " + to_string(round(ang_disp * 180 / M_PI)) +" deg");
    // lcd::set_text(2, "P_POS : ("  + to_string((int)round( 100* Get_Position().real())) + ", " + to_string((int)round( 100* Get_Position().imag())) + "), " + to_string((ang_disp * 180 / M_PI)) +" deg");

    // double sinTheta = sin(robotPos.angle); //Calculate the sine and cosine of the final angle to avoid redundancy
    // double cosTheta = cos(robotPos.angle);

    //Calculate updated absolute position values
    //Note that these specific signs and ratios are used because:
    // - dist is calculated using an angle clockwise from the positive y-axis (hence sin is x and cos is y); and
    // - dist2 is calculated using an angle clockwise from the positive x-axis (hence cos is x and -sin is y)
    // robotPos.x += dist * sinTheta + dist2 * cosTheta;
    // robotPos.y += dist * cosTheta - dist2 * sinTheta;

    //Print the tracking values to the brain screen for debugging
    // s__t(0, t__s(currentVal.left) + " " + t__s(currentVal.back));
    // s__t(1, t__s(robotPos.x) + " " + t__s(robotPos.y) + " " + t__s(robotPos.angle));
}

complex<double> Position_Tracker::Get_Position()
{
    auto initial_wheel_displacement = Position_Tracker::wheel_center_offset * exp<double>(1i * ang_origin);
    auto wheel_displacement = Position_Tracker::wheel_center_offset * exp<double>(1i * Get_Angle());
    return Get_Displacement() + origin;
    // return Get_Displacement() + origin - initial_wheel_displacement + wheel_displacement;
}

complex<double> Position_Tracker::Get_Position_N()
{
    auto initial_wheel_displacement = Position_Tracker::wheel_center_offset * exp<double>(1i * ang_origin);
    auto wheel_displacement = Position_Tracker::wheel_center_offset * exp<double>(1i * Get_Angle());
    return v_disp_n + h_disp_n + origin - initial_wheel_displacement + wheel_displacement;
}

complex<double> Position_Tracker::Get_Displacement()
{
    return pilons_disp;
}

complex<double> Position_Tracker::Get_Velocity()
{
    return pilonsVel;
}
// Position_Tracker::Poistion_Tra
