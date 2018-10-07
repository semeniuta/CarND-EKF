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

  // Predict the state

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;


}

void KalmanFilter::Update(const VectorXd& z) {

  // Update the state by using Kalman Filter equations

  VectorXd z_pred = H_ * x_;
  CommonUpdate(z, z_pred);

}

void KalmanFilter::UpdateEKF(const VectorXd& z) {

  // Update the state by using Extended Kalman Filter equations

  VectorXd z_pred = RadarMeasurementFunction();
  CommonUpdate(z, z_pred);

}

void KalmanFilter::CommonUpdate(const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred) {

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

Eigen::VectorXd KalmanFilter::RadarMeasurementFunction() {

  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px);
  double rho_dot = (px*vx + py*vy) / rho;

  Eigen::VectorXd res;
  res << rho, phi, rho_dot;

  return res;

}
