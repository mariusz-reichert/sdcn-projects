#include <type_traits>

#include "unscented_kalman_filter.hpp"
#include "sensor.hpp"
#include "tools.hpp"


namespace unscented_kalman_filter_project {

namespace { 

//Process noise standard deviation longitudinal acceleration in m/s^2
const double kSTD_A = 0.5;
//Process noise standard deviation yaw acceleration in rad/s^2
const double kSTD_YAWDD = 0.3*M_PI;
// Laser measurement noise standard deviation position1 in m
const double kSTD_LAS_PX = 0.15;
const double kLAS_PX_VAR = kSTD_LAS_PX*kSTD_LAS_PX;
// Laser measurement noise standard deviation position2 in m
const double kSTD_LAS_PY = 0.15;
const double kLAS_PY_VAR = kSTD_LAS_PY*kSTD_LAS_PY;
// Radar measurement noise standard deviation radius in m
const double kSTD_RAD_R = 0.3;
const double kRAD_R_VAR = kSTD_RAD_R*kSTD_RAD_R;
// Radar measurement noise standard deviation angle in rad
const double kSTD_RAD_PHI = 0.03;
const double kRAD_PHI_VAR = kSTD_RAD_PHI*kSTD_RAD_PHI;
// Radar measurement noise standard deviation radius change in m/s
const double kSTD_RAD_RD = 0.3;
const double kRAD_RD_VAR = kSTD_RAD_RD*kSTD_RAD_RD;
const int kNX = kINTERNAL_STATE_SPACE_SIZE;
const int kN_AUG = kNX+2;
const int kN_SIG = 2*kN_AUG+1;
// sigma point spreading parameter
constexpr double kLAMBDA = 3 - kN_AUG; 

}

using Eigen::VectorXd;
using std::underlying_type_t;
using namespace std;


UnscentedKalmanFilter::UnscentedKalmanFilter(const Measurement& initial_meas)
: x_{InternalState::Zero()}
, P_{(StateCovarianceMatrix() << 0.15, 0,    0,   0,   0,
                                 0,    0.15, 0,   0,   0,
                                 0,    0,    1,   0,   0, 
                                 0,    0,    0,   1,   0,
                                 0,    0,    0,   0,   1).finished()}
, Xsig_{kNX, 2*kNX+1}
, Xsig_aug_{kN_AUG, kN_SIG}
, Xsig_pred_{kNX, kN_SIG}
, weights_(kN_SIG)
, dt_{0.0}
, dt2_{0.0}
{
  if (initial_meas.source == SensorType::kLaser) {
    x_(0) = initial_meas.value(0);
    x_(1) = initial_meas.value(1);
  }
  else {
    const double rho = initial_meas.value(0);
    const double phi = initial_meas.value(1);
    const double rho_dot = initial_meas.value(2);

    const auto xy = tools::ConvertFromPolarToCartesian(rho, phi);
    x_(0) = xy.first;
    x_(1) = xy.second;
  }

  // set weights
  weights_(0) = kLAMBDA/(kLAMBDA+kN_AUG);
  for (int i=1; i<weights_.size(); ++i) {  //2n+1 weights
    weights_(i) = 0.5/(kN_AUG+kLAMBDA);
  }
}


State UnscentedKalmanFilter::GetStateEstimation() {
  State result{};
  const auto vx_vy = tools::ConvertFromPolarToCartesian(x_(2), x_(3));
  result << x_(0), x_(1), vx_vy.first, vx_vy.second;
  return result;
}


void UnscentedKalmanFilter::Predict(const double dt) {
  dt_ = dt;
  dt2_ = dt*dt;
  GenerateAugmentedSigmaPoints();
  SigmaPointPrediction();
  PredictMeanAndCovariance();
}


void UnscentedKalmanFilter::UdpateLidar(const VectorXd& z) {
  const int kNZ = kLASER_MEAS_SPACE_SIZE;
  //mean predicted measurement
  VectorXd z_pred = VectorXd(kNZ);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(kNZ,kNZ);
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(kNZ, kN_SIG);
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(kNX, kNZ);

  PredictLaserMeasurement(z_pred, S, Zsig);
  UpdateState(z_pred, S, Zsig, Tc, z, SensorType::kLaser);
}


void UnscentedKalmanFilter::UdpateRadar(const VectorXd& z) {
  const int kNZ = kRADAR_MEAS_SPACE_SIZE;
  //mean predicted measurement
  VectorXd z_pred = VectorXd(kNZ);
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(kNZ,kNZ);
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(kNZ, kN_SIG);
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(kNX, kNZ);

  PredictRadarMeasurement(z_pred, S, Zsig);
  UpdateState(z_pred, S, Zsig, Tc, z, SensorType::kRadar);
}

void UnscentedKalmanFilter::GenerateAugmentedSigmaPoints() {
  VectorXd x_aug = VectorXd(kN_AUG);
  MatrixXd P_aug = MatrixXd(kN_AUG, kN_AUG);

  //create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(kNX) = x_;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(kNX,kNX) = P_;
  P_aug(5,5) = kSTD_A*kSTD_A;
  P_aug(6,6) = kSTD_YAWDD*kSTD_YAWDD;

  //create square root matrix
  const MatrixXd L = P_aug.llt().matrixL();
  const auto C1 = sqrt(kLAMBDA+kN_AUG);

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  for (int i = 0; i< kN_AUG; ++i)
  {
    const auto C2 = C1 * L.col(i);
    Xsig_aug_.col(i+1)        = x_aug + C2;
    Xsig_aug_.col(i+1+kN_AUG) = x_aug - C2;
  }
}

void UnscentedKalmanFilter::SigmaPointPrediction() {
  //predict sigma points
  for (int i = 0; i < kN_SIG; ++i)
  {
    //extract values for better readability
    const auto & p_x = Xsig_aug_(0,i);
    const auto & p_y = Xsig_aug_(1,i);
    const auto & v = Xsig_aug_(2,i);
    const auto & yaw = Xsig_aug_(3,i);
    const auto & yawd = Xsig_aug_(4,i);
    const auto & nu_a = Xsig_aug_(5,i);
    const auto & nu_yawdd = Xsig_aug_(6,i);
    const auto sin_yaw = sin(yaw);
    const auto cos_yaw = cos(yaw);
    const auto yaw_changed = yaw+yawd*dt_;

    //predicted state values
    double px_p{0.0}, py_p{0.0};

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin(yaw_changed) - sin_yaw);
        py_p = p_y + v/yawd * ( cos_yaw - cos(yaw_changed) );
    }
    else {
        px_p = p_x + v*dt_*cos_yaw;
        py_p = p_y + v*dt_*sin_yaw;
    }

    double v_p = v;
    double yaw_p = yaw_changed;
    double yawd_p = yawd;

    //add noise
    px_p += 0.5*nu_a*dt2_ * cos_yaw;
    py_p += 0.5*nu_a*dt2_ * sin_yaw;
    v_p += nu_a*dt_;

    yaw_p += 0.5*nu_yawdd*dt2_;
    yawd_p += nu_yawdd*dt_;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
}

void UnscentedKalmanFilter::PredictMeanAndCovariance() {
  //predicted state mean
  x_ = Xsig_pred_ * weights_;

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < kN_SIG; ++i) {  //iterate over sigma points
    // state difference 
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    tools::NormalizeAngle(x_diff(3));

    P_ += weights_(i) * x_diff * x_diff.transpose() ;
  }
}

void UnscentedKalmanFilter::PredictRadarMeasurement(VectorXd& z_pred, 
  MatrixXd& S, 
  MatrixXd& Zsig) {
  const int kNZ = z_pred.size();

  //transform sigma points into measurement space
  for (int i = 0; i < kN_SIG; ++i) {  //2n+1 simga points
    // extract values for better readibility
    const double& p_x = Xsig_pred_(0,i);
    const double& p_y = Xsig_pred_(1,i);
    const double& v  = Xsig_pred_(2,i);
    const double& yaw = Xsig_pred_(3,i);
    const double& v1 = cos(yaw)*v;
    const double& v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);          //r
    Zsig(1,i) = atan2(p_y,p_x);                   //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / Zsig(0,i);   //r_dot
  }

  z_pred = Zsig*weights_;
  S.fill(0.0);

  for (int i = 0; i < kN_SIG; ++i) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    tools::NormalizeAngle(z_diff(1));
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(kNZ,kNZ);
  R << kRAD_R_VAR,            0,           0,
                0, kRAD_PHI_VAR,           0,
                0,            0, kRAD_RD_VAR;
  S += R;
}

void UnscentedKalmanFilter::PredictLaserMeasurement(VectorXd& z_pred, 
  MatrixXd& S, 
  MatrixXd& Zsig) {
  const int kNZ = kLASER_MEAS_SPACE_SIZE;

  //transform sigma points into measurement space
  for (int i = 0; i < kN_SIG; ++i) {
    // measurement model
    Zsig(0,i) = Xsig_pred_(0,i);                        //px
    Zsig(1,i) = Xsig_pred_(1,i);                        //py
  }

  z_pred = Zsig*weights_;
  S.fill(0.0);

  for (int i = 0; i < kN_SIG; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(kNZ,kNZ);
  R << kLAS_PX_VAR,           0,
                 0, kLAS_PY_VAR;
  S += R;
}

void UnscentedKalmanFilter::UpdateState(const VectorXd& z_pred, 
  const MatrixXd& S,
  const MatrixXd& Zsig,
  MatrixXd& Tc,
  const VectorXd& z,
  SensorType sensor) {
  
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < kN_SIG; ++i) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    if (sensor == SensorType::kRadar) {
      tools::NormalizeAngle(z_diff(1));
    }

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    tools::NormalizeAngle(x_diff(3));

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  if (sensor == SensorType::kRadar) {
    tools::NormalizeAngle(z_diff(1));
  }
  //update state mean and covariance matrix
  x_ += K * z_diff;
  P_ -= K*S*K.transpose();
}

} // namespace unscented_kalman_filter_project
