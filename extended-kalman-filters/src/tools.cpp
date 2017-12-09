#include <numeric>
#include <algorithm>
#include <functional>
#include <string>

#include "tools.hpp"

namespace extended_kalman_filter_project {

namespace tools {

using std::plus;
using std::make_pair;
using std::string;

State CalculateRmse(
    const vector<State>& estimations,
    const vector<State>& ground_truth) {
  const auto N = estimations.size();
  assert(N == ground_truth.size() and N > 0);

  auto SquareResiduals = [] (const auto& v1, const auto& v2) {
    const auto residuals = v1-v2;
    return residuals.array()*residuals.array();
  };

  State result = inner_product(estimations.begin(),
                               estimations.end(),
                               ground_truth.begin(),
                               result,
                               plus<State>(),
                               SquareResiduals);

  return (result / N).array().sqrt();
}


RadarMeasurementMatrix CalculateJacobian(const State& x) {
  const auto& px = x(0);
  const auto& py = x(1);
  const auto k1 = px*px+py*py;
  if (fabs(k1) < 0.0001) return RadarMeasurementMatrix::Zero();

  const auto& vx = x(2);
  const auto& vy = x(3);
  const auto k2 = sqrt(k1);
  const auto k3 = (k1*k2);

  RadarMeasurementMatrix Hj{};

  Hj <<  px/k2                , py/k2                , 0    , 0,
         -py/k1               , px/k1                , 0    , 0,
         py*(vx*py - vy*px)/k3, px*(px*vy - py*vx)/k3, px/k2, py/k2;

  return Hj;

}


VectorXd h(const State& x) {
  const auto& px = x(0);
  const auto& py = x(1);
  const auto& vx = x(2);
  const auto& vy = x(3);
  const auto k1 = px*px+py*py;

  if (fabs(k1) < 0.0001) {
    return (VectorXd(kRADAR_MEAS_SPACE_SIZE) << 0, 0, 0).finished();
  }
  const auto rho = sqrt(k1);
  return (VectorXd(kRADAR_MEAS_SPACE_SIZE) << rho, 
                                              atan2(py,px), 
                                              (px*vx+py*vy)/rho).finished();
};


pair<double, double> ConvertFromPolarToCartesian(double rho, double phi) {
  return make_pair(rho * cos(phi), rho * sin(phi));
}


void CheckArgs(int argc, char* argv[]) {
  string error{""};
  string usage{"Usage instructions: "};
  usage += argv[0];
  usage += " path/to/input.txt output.txt";
  bool has_valid_args{false};

  switch(argc) {
    case 1:
      error = "Input and output files are not provided.\n";
      break;
    case 2:
      error = "Output file is not provided.\n";
      break;
    case 3:
      has_valid_args = true;
      break;
    default:
      error = "Too many arguments given.\n";
  }

  if (not has_valid_args) {
    cerr << error + usage << endl;
    exit(EXIT_FAILURE);
  }
}

} // namespace tools

} // namespace extended_kalman_filter_project