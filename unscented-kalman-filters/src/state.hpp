#pragma once

#include <iosfwd>

#include "Eigen/Dense"


namespace unscented_kalman_filter_project {

using std::istream;
using std::ostream;
using Eigen::Matrix;


// <px, py, vx, vy>
constexpr int kSTATE_SPACE_SIZE = 4;
using State = Matrix<double, kSTATE_SPACE_SIZE, 1>;

// <px, py, v, psi, psi_dot>
constexpr int kINTERNAL_STATE_SPACE_SIZE = 5;
using InternalState = Matrix<double, kINTERNAL_STATE_SPACE_SIZE, 1>;


istream& operator>>(istream& ins, State& x);
ostream& operator<<(ostream& outs, const State& x);

} // namespace unscented_kalman_filter_project
