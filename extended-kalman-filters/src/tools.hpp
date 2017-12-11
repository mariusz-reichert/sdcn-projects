#pragma once

#include <vector>
#include <iostream>

#include "Eigen/Dense"
#include "state.hpp"
#include "kalman_filter_matrixes.hpp"

namespace extended_kalman_filter_project {

namespace tools {

using std::pair;
using std::endl;
using std::cerr;
using std::vector;
using Eigen::VectorXd;


State CalculateRmse(
  const vector<State>& estimations, 
  const vector<State>& ground_truth);


void NormalizeAngle(double& angle) noexcept;


RadarMeasurementMatrix CalculateJacobian(const State& x);


// function that specifies how the predicted position and speed relate to range, 
// bearing and range rate
VectorXd h(const State& x);


pair<double, double> ConvertFromPolarToCartesian(double rho, double phi);


void CheckArgs(int argc, char* argv[]);


template <typename T>
void CheckFile(const T& file, const char* name) {
  if (not file.is_open()) {
    cerr << "Cannot open input file: " << name << endl;
    exit(EXIT_FAILURE);
  }
}

} // namespace tools

} // namespace extended_kalman_filter_project