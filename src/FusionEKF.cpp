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
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement noise covariance matrix - laser, from "Lesson 5: 13 Laser Measurements Part 4"
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement noise covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Laser measurement function. From "Lesson 5: 10. Laser Measurements Part 1"
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Hj_ is state-dependent, calculate on-demand by Tools:CalculateJacobian(x_state)

  // TODO Get initial X from data set 1 to reduce error
  VectorXd x_in = VectorXd(4);
  x_in << 0, 0, 0, 0;

  // Initial state covariance matrix from "Lesson 5: 13. Laser Measurements Part 4"
  MatrixXd P_in = MatrixXd(4, 4);
  P_in << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

  // State transition matrix, from "Lesson 5: 13 Laser Measurements Part 4"
  MatrixXd F_in = MatrixXd(4, 4);
  F_in <<  1, 0, 1, 0,
           0, 1, 0, 1,
           0, 0, 1, 0,
           0, 0, 0, 1;

  // Initial process covariance matrix, depends on delta-t, noise_ax and noise_ay,
  // so let's just pretend delta_T is 0 here which results in all-zeroes, then update to actual value later
  MatrixXd Q_in = MatrixXd(4, 4);
  Q_in.setZero();


  ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);

  // TODO set noise and calculate Q somewhere, see Lesson 5: 13 Laser m. Part 4
  // set the acceleration noise components, see Lesson 5 section 13
  // noise_ax = 5; // Q&A video says 9 (=3 squared), lesson 5: 13 says 5
  // noise_ay = 5; // Q&A video says 9, lesson says 5

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

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
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double ro = measurement_pack.raw_measurements_(0)
      double theta = measurement_pack.raw_measurements_(1)
      double ro_dot = measurement_pack.raw_measurements_(2)
      long long ts = measurement_pack.timestamp_
      ekf_.x_(0) = ro * cos(theta);
      ekf_.x_(1) = ro * sin(theta);
      ekf_.x_(2) = ro_dot * cos(theta);
      ekf_.x_(3) = ro_dot * sin(theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      double px = measurement_pack.raw_measurements_(0)
      double py = measurement_pack.raw_measurements_(1)
      long long ts = measurement_pack.timestamp_
      ekf_.x_(0) = px;
      ekf_.x_(1) = py;
      // TODO in case of laser, guess velocity to reduce RMSE
      ekf_.x_(2) = 1;
      ekf_.x_(3) = 1;
    }

    // Initialize the state transition matrix F, initially identity matrix
    ekf_.F_.setIdentity();

    previous_timestamp_ = measurement_pack.timestamp_;
    // TODO continue Q&A from minute 24

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // TODO finish this and double-check it...
  float dt - (measurement_pack.timestamp - previous_timestamp_) / 1000000

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
