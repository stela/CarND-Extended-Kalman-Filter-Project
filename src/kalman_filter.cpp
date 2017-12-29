#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/**
 * Init Initializes Kalman filter
 * @param x_in Initial state
 * @param P_in Initial state covariance
 * @param F_in Transition matrix
 * @param H_in Measurement matrix
 * @param R_in Measurement covariance matrix
 * @param Q_in Process covariance matrix
 */
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/**
 * Prediction Predicts the state and the state covariance
 * using the process model
 * @param delta_T Time between k and k+1 in s
 */
void KalmanFilter::Predict() {
    // u = external motion, assume always zero for now
    auto u = VectorXd::Zero(x_.rows());

    // code originally from "Lesson 5: Lidar and Radar Fusion with Kalman Filters"
    // KF Prediction step
    x_ = F_ * x_ + u;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

    std::cout << "x=" << std::endl <<  x_ << std::endl;
    std::cout << "P=" << std::endl <<  P_ << std::endl;
}

/**
 * Updates the state by using standard Kalman Filter equations
 * @param z The measurement at k+1 (for lidar, only position, not velocity)
 */
void KalmanFilter::Update(const VectorXd &z) {
  auto I = MatrixXd::Identity(F_.cols(), F_.rows());

  // code originally from "Lesson 5: Lidar and Radar Fusion with Kalman Filters"
  // KF Measurement update step

  VectorXd y = z - H_ * x_;     // measurement-space difference of measurement and prediction
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  // TODO instead of multiplying with inverse of S, faster and more stable to solve S x' = P Ht x
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

/**
 * Updates the state by using Extended Kalman Filter equations
 * @param z The measurement at k+1
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
