#pragma once

#include <vector>

#include "Eigen-3.3/Eigen/Core"

namespace model_predictive_control_project {

using namespace std;
using Eigen::VectorXd;

struct MPC {
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(const VectorXd& state, const VectorXd& coeffs);
};

} // namespace model_predictive_control_project
