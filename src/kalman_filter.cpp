#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

void KalmanFilter::Init(VectorXd& x_in, MatrixXd& P_in, MatrixXd& F_in,
                        MatrixXd& H_in, MatrixXd& R_in, MatrixXd& Q_in) {
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

void KalmanFilter::Update(const VectorXd& z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
}

void KalmanFilter::UpdateEKF(const VectorXd& z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}

Eigen::VectorXd KalmanFilter::RadarMeasurementFunction() {

  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px*px + py*py);
  double phi = atan(py / px);
  double rho_dot = (px*vx + py*vy) / rho;

  Eigen::VectorXd res;
  res << rho, phi, rho_dot;

  return res;

}
