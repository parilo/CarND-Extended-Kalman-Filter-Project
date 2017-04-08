#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  ekf_.R_kf_  = MatrixXd(2, 2);
  ekf_.R_ekf_ = MatrixXd(3, 3);
  ekf_.H_kf_  = MatrixXd(2, 4);
//  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  ekf_.R_kf_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  ekf_.R_ekf_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<  1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  //measurement matrix
  ekf_.H_kf_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // order in measurements: rho, phi, rho_dot
      double rho = measurement_pack.raw_measurements_ [0];
      double tg_phi = tan (measurement_pack.raw_measurements_ [1]);
      double x = rho / std::sqrt (tg_phi*tg_phi + 1) / tg_phi;
      double y = rho / std::sqrt (tg_phi*tg_phi + 1);
      ekf_.x_ << x, y, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_ [0], measurement_pack.raw_measurements_ [1], 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  //1. Modify the F matrix so that the time is integrated
  ekf_.F_ (0, 2) = dt;
  ekf_.F_ (1, 3) = dt;
  //2. Set the process covariance matrix Q
  ekf_.Q_ = MatrixXd (4, 4);
  float dt_2 = dt*dt;
  float dt_3 = dt_2*dt/2;
  float dt_4 = dt_3*dt/2;
  ekf_.Q_ << noise_ax*dt_4, 0, noise_ax*dt_3, 0,
             0, noise_ay*dt_4, 0, noise_ay*dt_3,
             noise_ax*dt_3, 0, noise_ax*dt_2, 0,
             0, noise_ay*dt_3, 0, noise_ay*dt_2;
  //3. Call the Kalman Filter predict() function
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
