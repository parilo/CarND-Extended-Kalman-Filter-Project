#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
  I_ = MatrixXd (4, 4);
  I_.setIdentity ();
}

KalmanFilter::~KalmanFilter() {}

//void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
//                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
//  x_ = x_in;
//  P_ = P_in;
//  F_ = F_in;
//  H_ = H_in;
//  R_ = R_in;
//  Q_ = Q_in;
//}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose ();
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
  Eigen::VectorXd y = z - tools.CalculateRadarMeasurement(x_);
  Eigen::MatrixXd H_ekf = tools.CalculateJacobian(x_);
  Eigen::MatrixXd S = H_ekf * P_ * H_ekf.transpose () + R_ekf_;
  Eigen::MatrixXd K = P_ * H_ekf.transpose () * S.inverse ();
  x_ = x_ + K * y;
  P_ = (I_ - K * H_ekf) * P_;
}
