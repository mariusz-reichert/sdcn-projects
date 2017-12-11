#include <iostream>
#include <fstream>
#include <tuple>
#include <utility>
#include <sstream>

#include "tools.hpp"
#include "measurement.hpp"
#include "state.hpp"
#include "sensor_fusion.hpp"

using namespace extended_kalman_filter_project;

namespace {

auto GetValidatedFiles(int argc, char* argv[]) {
  using namespace std;

  tools::CheckArgs(argc, argv);

  ifstream fin{argv[1], ifstream::in};
  tools::CheckFile(fin, argv[1]);

  ofstream fout{argv[2], ofstream::out};
  tools::CheckFile(fout, argv[2]);

  return make_pair(move(fin), move(fout));
}

} // namespace


int main(int argc, char* argv[]) {
  using namespace std;
  
  ifstream fin{};
  ofstream fout{};
  tie(fin, fout) = GetValidatedFiles(argc, argv);

  string line{};
  string sensor_type{};
  istringstream iss{};

  vector<Measurement> measurements{};
  vector<State> ground_truths{};
  vector<State> estimations{};

  while (getline(fin, line)) {
    iss.str(line);
    iss >> sensor_type;

    Measurement m{sensor_type == "L" ? SensorType::kLaser : SensorType::kRadar};
    iss >> m;
    measurements.push_back(move(m));

    State s{};
    iss >> s;
    ground_truths.push_back(move(s));

    iss.clear();
  }
  const auto N = measurements.size();
  assert(N > 0u);

  // initialize kalman filter with the first measurement
  SensorFusionEkf filter{measurements.front()};
  estimations.push_back(filter.GetStateEstimation());

  // process the rest of measurements in normal predict-update loop
  for (size_t i = 1u; i < N; ++i) {
    const auto estimation = filter.Process(measurements[i]);

    fout << estimation 
         << measurements[i] 
         << ground_truths[i] 
         << endl;

    estimations.push_back(move(estimation));
  }

  cout << "Accuracy - RMSE:\t" 
       << tools::CalculateRmse(estimations, ground_truths) << endl;

  return 0;
}
