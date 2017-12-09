#pragma once

#include "state.hpp"
#include "measurement.hpp"
#include "unscented_kalman_filter.hpp"


namespace unscented_kalman_filter_project {

constexpr double kMICROSECONDS_IN_SECOND = 1000000.0;


class SensorFusionUkf {
  UnscentedKalmanFilter ukf_;
  long long previous_timestamp_;

public:
  explicit SensorFusionUkf(const Measurement& initial_meas)
  : ukf_{initial_meas}
  , previous_timestamp_{initial_meas.timestamp}
  {}

  State GetStateEstimation() { return ukf_.GetStateEstimation(); }

  State Process(const Measurement& m) {
    ukf_.Predict((m.timestamp - previous_timestamp_)/kMICROSECONDS_IN_SECOND);

    if (m.source == SensorType::kLaser) {
      ukf_.UdpateLidar(m.value);
    }
    else {
      ukf_.UdpateRadar(m.value);
    }

    previous_timestamp_ = m.timestamp;
    return GetStateEstimation();
  }
};

} // namespace unscented_kalman_filter_project
