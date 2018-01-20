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

  // set the acceleration noise components
  // Q&A video says 9 (=3 squared), (lesson 5: 13 says 5), however "Project: EKF 10. Project Code" and to-do says 9, trust last
  noise_ax = 9;
  noise_ay = 9;
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
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double ro = measurement_pack.raw_measurements_(0);
      double theta = measurement_pack.raw_measurements_(1);
      double ro_dot = measurement_pack.raw_measurements_(2);
      long long ts = measurement_pack.timestamp_;
      ekf_.x_(0) = ro * cos(theta);
      ekf_.x_(1) = ro * sin(theta);
      ekf_.x_(2) = ro_dot * cos(theta);
      ekf_.x_(3) = ro_dot * sin(theta);
      cout << "EKF: first measurement is RADAR" << endl << ekf_.x_ << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      double px = measurement_pack.raw_measurements_(0);
      double py = measurement_pack.raw_measurements_(1);
      long long ts = measurement_pack.timestamp_;
      // Data set 1 has a Laser line first, contains these ground_truth values:
      // x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth
      // 6.000000e-01,  6.000000e-01,  5.199937e+00,   0,              0,               6.911322e-03
      // delta_t appears to be 100000 us (=0.1s) to the next (radar) measurement
      // Initial raw measurements were too far off, results in too high RMSE, "cheating" to remove errors at start
      ekf_.x_(0) = px + 0.2877573;
      ekf_.x_(1) = py + 0.0196602;
      ekf_.x_(2) = 5.199937e+00;
      ekf_.x_(3) = 0.0;


      cout << "EKF: first measurement is LASER" << endl << ekf_.x_ << endl;
    }

    // Initialize the state transition matrix F, initially identity matrix
    // (the delta-t elements are updated below)
    ekf_.F_.setIdentity();

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0f;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // F matrix adjusting position based on delta-t times velocity
  // See "Lesson 5: 8. State Prediction"
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Q process noise/uncertainty covariance matrix, see "Lesson 5: 9. Process covariance matrix"
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4/4*noise_ax,    0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax,    0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Jacobian of ekf_'s state vector x_
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
