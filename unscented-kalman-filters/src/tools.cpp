#include <numeric>
#include <algorithm>
#include <functional>
#include <string>

#include "tools.hpp"

namespace unscented_kalman_filter_project {

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

void NormalizeAngle(double& angle) {
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
}

} // namespace tools

} // namespace unscented_kalman_filter_project