#pragma once

#include <type_traits>
#include <iosfwd>

#include "Eigen/Dense"
#include "sensor.hpp"
#include "tools.hpp"

namespace unscented_kalman_filter_project {

using std::istream;
using std::ostream;
using std::underlying_type_t;
using Eigen::VectorXd;


struct Measurement {
  const SensorType source;
  VectorXd value;
  long long timestamp;

  explicit Measurement(SensorType s) 
  : source{s} 
  , value{static_cast<underlying_type_t<SensorType>>(source)}
  , timestamp{0}
  {} 
};


istream& operator>>(istream& ins, Measurement& m);
ostream& operator<<(ostream& outs, const Measurement& m);

} // namespace unscented_kalman_filter_project
