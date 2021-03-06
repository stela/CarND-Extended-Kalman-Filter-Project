#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

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

  // code originally from "Lesson 13: Laser measurements part 4"
  // KF Prediction step
  // Assuming no external motion "u" to add, assuming u is always zero for now
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  // Q_ is dependent on delta_T, see Lesson 15: 13 Laser m. part 4
  // but that's taken care of by FusionEKF before this method is called
  P_ = F_ * P_ * Ft + Q_;

  // std::cout << "x=" << std::endl <<  x_ << std::endl;
  // std::cout << "P=" << std::endl <<  P_ << std::endl;
}

// There's no need for a PredictEKF function, since the prediction model is linear

/**
 * Updates the state by using standard Kalman Filter equations
 * @param z The measurement at k+1 (for lidar, only position, not velocity)
 */
void KalmanFilter::Update(const VectorXd &z) {
  // code originally from "Lesson 13: Laser measurements part 4"
  // H matrix works for lidar (linear) but not for radar (polar coordinates), use h(x) function for radar,
  // but still works after linearizing h(x), using e.g. first order Taylor expansion
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;  // measurement-space difference of measurement and prediction
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  // instead of multiplying with inverse of S, faster and more stable to solve S x' = P Ht x
  // See https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;


  // new state estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  // Q&A video plus "Lesson 5: 7. Kalman F. Eq. part 2". contained a prediction-step here, but that was an accident, right?
}

// Only radar (not lidar) updates have a non-linear model and require use of EKF. Lidar-updates are linear.

static double normalize_phi(double phi) {
  while (phi > M_PI || phi < -M_PI) {
    if (phi > M_PI) {
      phi -= M_PI;
    }
    if (phi < -M_PI) {
      phi += M_PI;
    }
  }
  return phi;
}

/**
 * Updates the radar state by using Extended Kalman Filter equations
 * @param z The measurement at k+1
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Like the regular Update() except when called, H_ is already assigned to the jacobian,
  // and z_pred = h(x) needs to be calculated manually.
  // Details in "Lesson 5: 14. Radar measurements"

  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px*px + py*py);
  double theta = atan2(py, px);
  double ro_dot = (px*vx + py*vy)/rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;

  // the rest of the equations are like above in Update()...
  VectorXd y = z - z_pred;  // measurement-space difference of measurement and prediction
  // normalize angle phi in y again here. Otherwise a "pirouette" is seen half-ways
  y(1) = normalize_phi(y(1));


  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  // instead of multiplying with inverse of S, faster and more stable to solve S x' = P Ht x
  // See https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new state estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  // Q&A video plus "Lesson 5: 7. Kalman F. Eq. part 2". contained a prediction-step here, but that was an accident, right?
}

