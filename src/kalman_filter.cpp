#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
  I_ = MatrixXd (4, 4);
  I_.setIdentity ();
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose () + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  Eigen::VectorXd y = z - H_kf_ * x_;
  Eigen::MatrixXd S = H_kf_ * P_ * H_kf_.transpose () + R_kf_;
  Eigen::MatrixXd K = P_ * H_kf_.transpose () * S.inverse ();
  x_ = x_ + K * y;
  P_ = (I_ - K * H_kf_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  Eigen::MatrixXd H_ekf;
  Eigen::VectorXd y;
  if ((x_(0)*x_(0) + x_(1)*x_(1)) == 0) {
    H_ekf.setIdentity (3, 4);
    H_ekf (2, 3) = 1;
    y = z;
  } else {
    H_ekf = tools.CalculateJacobian(x_);
    y = z - tools.CalculateRadarMeasurement(x_);
  }

  y (1) = fmod(y (1) + M_PI, 2*M_PI) - M_PI;
  Eigen::MatrixXd S = H_ekf * P_ * H_ekf.transpose () + R_ekf_;
  Eigen::MatrixXd K = P_ * H_ekf.transpose () * S.inverse ();
  x_ = x_ + K * y;
  P_ = (I_ - K * H_ekf) * P_;
}
