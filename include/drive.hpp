#ifndef __DRIVE__H__
#define __DRIVE__H__

#include <array>
const std::array<double,2> Calc_Drive_MPC(double left, double right, bool print = false);

#endif  //!__DRIVE__H__