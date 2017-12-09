#pragma once

#include "state.hpp"
#include "measurement.hpp"
#include "Eigen/Dense"


namespace unscented_kalman_filter_project {

using Eigen::VectorXd;
using Eigen::MatrixXd;

using StateCovarianceMatrix = Matrix<double, 
                                     kINTERNAL_STATE_SPACE_SIZE, 
                                     kINTERNAL_STATE_SPACE_SIZE>;

class UnscentedKalmanFilter {
  InternalState x_;
  StateCovarianceMatrix P_;
  MatrixXd Xsig_;
  MatrixXd Xsig_aug_;
  MatrixXd Xsig_pred_;
  VectorXd weights_;
  double dt_;
  double dt2_;
public:
  explicit UnscentedKalmanFilter(const Measurement& initial_meas);
  State GetStateEstimation();
  void Predict(const double dt);
  void UdpateLidar(const VectorXd& z);
  void UdpateRadar(const VectorXd& z);
private:
  void GenerateAugmentedSigmaPoints();
  void SigmaPointPrediction();
  void PredictMeanAndCovariance();

  void PredictRadarMeasurement(VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig);
  void PredictLaserMeasurement(VectorXd& z_pred, MatrixXd& S, MatrixXd& Zsig);

  void UpdateState(const VectorXd& z_pred, 
                   const MatrixXd& S, 
                   const MatrixXd& Zsig,
                   MatrixXd& Tc,
                   const VectorXd& z,
                   SensorType sensor);
};


} // namespace unscented_kalman_filter_project