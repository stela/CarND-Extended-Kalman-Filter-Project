#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
}

/**
 * Update filter state based on new measurement z.
 * @param z new measurement
 */
void KalmanFilter::Update(const VectorXd &z) {
  auto I = MatrixXd::Identity(F_.cols(), F_.rows());
  auto u = VectorXd(x_.rows());

  // code originally from "Lesson 5: Lidar and Radar Fusion with Kalman Filters"

  // KF Measurement update step
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  // TODO instead of multiplying with inverse of S, faster and more stable to solve S x' = P Ht x
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;

  // KF Prediction step
  x_ = F_ * x_ + u;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

  std::cout << "x=" << std::endl <<  x_ << std::endl;
  std::cout << "P=" << std::endl <<  P_ << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
