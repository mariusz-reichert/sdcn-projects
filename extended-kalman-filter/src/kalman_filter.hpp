#pragma once

#include "state.hpp"
#include "measurement.hpp"
#include "kalman_filter_matrixes.hpp"

namespace extended_kalman_filter_project {

using Eigen::VectorXd;

constexpr double kNOISE_AX{9};
constexpr double kNOISE_AY{9};


class ExtendedKalmanFilter {
  State x_;
  StateCovarianceMatrix P_;
  StateTransitionMatrix F_;
  ProcessCovarianceMatrix Q_;
  const LaserMeasurementMatrix laser_H_;
  const LaserMeasurementCovarianceMatrix laser_R_;
  const RadarMeasurementCovarianceMatrix radar_R_;

public:

  explicit ExtendedKalmanFilter(const Measurement& initial_meas);

  State GetStateEstimation() noexcept { return x_; }

  void Predict(const double dt);

  void Udpate(const VectorXd& z);

  void UdpateEkf(const VectorXd& z);
};

} // namespace extended_kalman_filter_project
