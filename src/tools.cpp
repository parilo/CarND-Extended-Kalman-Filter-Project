#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // code taken from my lesson quiz answer
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  assert(estimations.size() != 0);
  assert(estimations.size() == ground_truth.size());

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
      VectorXd d = estimations [i] - ground_truth [i];
      VectorXd e2 = d.array() * d.array();
      rmse += e2;
  }

  //calculate the mean
  rmse /= 1.0*estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // code taken from my lesson quiz answer

  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);


  float sq = std::sqrt(px*px + py*py);
  float sq2 = sq*sq;
  float sq3 = sq2*sq;

  //check division by zero
  if (sq == 0) {
      std::cout << "error: Division ny zero" << std::endl;
      return Hj;
  }

  //compute the Jacobian matrix
  Hj << px/sq, py/sq, 0, 0,
        -py/sq2, px/sq2, 0, 0,
        py*(vx*py - vy*px)/sq3, px*(- vx*py + vy*px)/sq3, px/sq, py/sq;

  return Hj;
}

Eigen::VectorXd Tools::CalculateRadarMeasurement (const Eigen::VectorXd& x_state) {
  float sq = std::sqrt (x_state (0) * x_state (0) + x_state (1) * x_state (1));
  VectorXd r = VectorXd (3);
  r (0) = sq;
  r (1) = std::atan2 (x_state (1), x_state (0));
  r (2) = (x_state (0) * x_state (2) + x_state (1) * x_state (3)) / sq;
  return r;
}
