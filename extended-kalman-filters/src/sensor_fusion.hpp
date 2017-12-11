#pragma once

#include "state.hpp"
#include "measurement.hpp"
#include "kalman_filter.hpp"


namespace extended_kalman_filter_project {

constexpr double kMICROSECONDS_IN_SECOND = 1000000.0;


class SensorFusionEkf {
  ExtendedKalmanFilter ekf_;
  long long previous_timestamp_;

public:
  explicit SensorFusionEkf(const Measurement& initial_meas)
  : ekf_{initial_meas}
  , previous_timestamp_{initial_meas.timestamp}
  {}

  const State& GetStateEstimation() noexcept { return ekf_.GetStateEstimation(); }

  void Process(const Measurement& m) {
    const auto dt = (m.timestamp - previous_timestamp_)/kMICROSECONDS_IN_SECOND;

    if (dt > 0.001) {
      ekf_.Predict(dt);
      if (m.source == SensorType::kLaser) {
        ekf_.Udpate(m.value);
      }
      else {
        ekf_.UdpateEkf(m.value);
      }
      previous_timestamp_ = m.timestamp;
    }
  }
};

} // namespace extended_kalman_filter_project
