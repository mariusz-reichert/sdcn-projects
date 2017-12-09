#pragma once

#include "Eigen/Dense"
#include "state.hpp"
#include "sensor.hpp"

namespace extended_kalman_filter_project {

using Eigen::Matrix;

using StateCovarianceMatrix = Matrix<double, 
                                     kSTATE_SPACE_SIZE, 
                                     kSTATE_SPACE_SIZE>;
using StateTransitionMatrix = Matrix<double, 
                                     kSTATE_SPACE_SIZE, 
                                     kSTATE_SPACE_SIZE>;   
using ProcessCovarianceMatrix = Matrix<double, 
                                       kSTATE_SPACE_SIZE, 
                                       kSTATE_SPACE_SIZE>;                                                                                                             
using LaserMeasurementMatrix = Matrix<double, 
                                      kLASER_MEAS_SPACE_SIZE, 
                                      kSTATE_SPACE_SIZE>;
using RadarMeasurementMatrix = Matrix<double, 
                                      kRADAR_MEAS_SPACE_SIZE, 
                                      kSTATE_SPACE_SIZE>;
using LaserMeasurementCovarianceMatrix = Matrix<double, 
                                                kLASER_MEAS_SPACE_SIZE, 
                                                kLASER_MEAS_SPACE_SIZE>;
using RadarMeasurementCovarianceMatrix = Matrix<double, 
                                                kRADAR_MEAS_SPACE_SIZE, 
                                                kRADAR_MEAS_SPACE_SIZE>;

} // namespace extended_kalman_filter_project
