#ifndef __FUNCTIONAL_TASKS__H__
#define __FUNCTIONAL_TASKS__H__

#include <complex>
#include <array>

#include "auto_task.hpp"
#include "pathing.hpp"
#include "control/state_space_methods.hpp"

extern AutoTask AutoPath(std::complex<double> EndPoint, double angle, double speed, double tolerance);
extern AutoTask AutoPath(std::complex<double> EndPoint, double angle, double speed, std::array<double, 2> tolerance = {1, 1});
extern AutoTask AutoPath(std::complex<double> EndPoint, double angle, std::array<double, 2> speed = {100, 100}, double tolerance = 1);
extern AutoTask AutoPath(std::complex<double> EndPoint, double angle, std::array<double, 2> speed = {100, 100}, std::array<double, 2> tolerance = {1, 1});
extern AutoTask SingleRun(std::function<void(void)> run);


extern AutoTask AutoCurve(std::complex<double> Waypoint, double angle, std::complex<double> EndPoint, double endAngle, double speed, double curvature = 1, double angleInterpolation = 1);
extern AutoTask TurnToPointTask(Point target, double maxError = 0.07, double turnSpeed = 127);
extern AutoTask DriveProfileTask(double speed, double time = 1000);
extern AutoTask RestProfileTask();

extern AutoTask IntakeShootTask(int numBallsOut);
extern AutoTask PurePursuitTask(std::complex<double> EndPoint, double EndAngle, double speed, std::array<double, 2> errorTolerance = {0,0});

extern AutoTask TurnToPointSmooth(Point targetPoint, double accel = 0.5, double errorTolerance = 0.1);
extern AutoTask PurePursuitSmooth(Point targetPoint, double accel = 0.5, double errorTolerance = 1);
extern AutoTask PurePursuitSimple(const Point targetPoint, const double accel = 0.5, const double errorTolerance = 0.5);


#endif //!__FUNCTIONAL_TASKS__H__
