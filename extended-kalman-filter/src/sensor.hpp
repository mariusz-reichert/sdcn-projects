#pragma once

namespace extended_kalman_filter_project {

constexpr unsigned kLASER_MEAS_SPACE_SIZE = 2u;
constexpr unsigned kRADAR_MEAS_SPACE_SIZE = 3u;


enum class SensorType : unsigned {
  kLaser = kLASER_MEAS_SPACE_SIZE,
  kRadar = kRADAR_MEAS_SPACE_SIZE
};

} // namespace extended_kalman_filter_project
