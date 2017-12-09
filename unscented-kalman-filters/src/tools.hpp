#pragma once

#include <vector>
#include <iostream>

#include "Eigen/Dense"
#include "state.hpp"

namespace unscented_kalman_filter_project {

namespace tools {

using std::pair;
using std::endl;
using std::cerr;
using std::vector;


State CalculateRmse(
  const vector<State>& estimations, 
  const vector<State>& ground_truth);


pair<double, double> ConvertFromPolarToCartesian(double rho, double phi);


void CheckArgs(int argc, char* argv[]);


template <typename T>
void CheckFile(const T& file, const char* name) {
  if (not file.is_open()) {
    cerr << "Cannot open input file: " << name << endl;
    exit(EXIT_FAILURE);
  }
}

void NormalizeAngle(double& angle);

} // namespace tools

} // namespace unscented_kalman_filter_project