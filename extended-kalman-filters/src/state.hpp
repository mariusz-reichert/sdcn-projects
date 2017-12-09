#pragma once

#include <iosfwd>

#include "Eigen/Dense"


namespace extended_kalman_filter_project {

using std::istream;
using std::ostream;
using Eigen::Matrix;


constexpr size_t kSTATE_SPACE_SIZE = 4u;
using State = Matrix<double, kSTATE_SPACE_SIZE, 1>;


istream& operator>>(istream& ins, State& x);
ostream& operator<<(ostream& outs, const State& x);

} // namespace extended_kalman_filter_project
