#include "kalman_filter.hpp"
#include "sensor.hpp"
#include "tools.hpp"


namespace extended_kalman_filter_project {

using Eigen::VectorXd;


ExtendedKalmanFilter::ExtendedKalmanFilter(const Measurement& initial_meas)
: x_{State::Zero()}
, P_{StateCovarianceMatrix::Identity()}
, F_{StateTransitionMatrix::Identity()}
, Q_{ProcessCovarianceMatrix::Zero()}
, laser_H_{(LaserMeasurementMatrix() << 1, 0, 0, 0,
                                        0, 1, 0, 0).finished()}
, laser_R_{(LaserMeasurementCovarianceMatrix() <<  0.0225, 0,
                                                   0     , 0.0225).finished()}
, radar_R_{(RadarMeasurementCovarianceMatrix() << 0.09, 0     , 0,
                                                  0   , 0.0009, 0,
                                                  0   , 0     , 0.09).finished()}
{
  if (initial_meas.source == SensorType::kLaser) {
    x_(0) = initial_meas.value(0);
    x_(1) = initial_meas.value(1);
  }
  else {
    const auto& rho = initial_meas.value(0);
    double phi = initial_meas.value(1);
    tools::NormalizeAngle(phi);
    const auto& rho_dot = initial_meas.value(2);
    const auto xy = tools::ConvertFromPolarToCartesian(rho, phi);
    x_(0) = xy.first;
    x_(1) = xy.second;
    x_(2) = rho_dot * cos(phi);
    x_(3) = rho_dot * sin(phi);
  }
}


void ExtendedKalmanFilter::Predict(const double dt) {
  F_(0,2) = dt;
  F_(1,3) = dt;
  const auto Ft = F_.transpose();
  const auto dt2 = dt * dt;
  const auto dt3 = dt2 * dt;
  const auto dt4 = dt3 * dt;

  Q_ << dt4/4*kNOISE_AX, 0              , dt3/2*kNOISE_AX, 0,
        0              , dt4/4*kNOISE_AY, 0              , dt3/2*kNOISE_AY,
        dt3/2*kNOISE_AX, 0              , dt2*kNOISE_AX  , 0,
        0              , dt3/2*kNOISE_AY, 0              , dt2*kNOISE_AY;

  x_ = F_ * x_;
  P_ = F_ * P_ * Ft + Q_;
}


void ExtendedKalmanFilter::Udpate(const VectorXd& z) {
  const auto& H = laser_H_;
  const auto& R = laser_R_;
  const auto z_pred = H * x_;

  const auto Ht = H.transpose();
  const auto I = StateCovarianceMatrix::Identity();

  const auto y = z - z_pred;
  const auto S = H * P_ * Ht + R;
  const auto Si = S.inverse();
  const auto PHt = P_ * Ht;
  const auto K = PHt * Si;

  x_ = x_ + (K * y);
  P_ = (I - K * H) * P_;
}


void ExtendedKalmanFilter::UdpateEkf(const VectorXd& z) {
  const auto H = tools::CalculateJacobian(x_);
  const auto& R = radar_R_;
  const auto z_pred = tools::h(x_);

  const auto Ht = H.transpose();
  const auto I = StateCovarianceMatrix::Identity();

  const auto y = z - z_pred;
  const auto S = H * P_ * Ht + R;
  const auto Si = S.inverse();
  const auto PHt = P_ * Ht;
  const auto K = PHt * Si;

  x_ = x_ + (K * y);
  P_ = (I - K * H) * P_;
}

} // namespace extended_kalman_filter_project
