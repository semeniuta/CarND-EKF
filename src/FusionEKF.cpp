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

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement function - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;


  // state transition function
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  // Values at (0, 2) and (1, 3) are changed to dt
  // before every call of ekf_.Predict

}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  if (!is_initialized_) { // first measurement

    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      // Convert radar from polar to cartesian coordinates and initialize state.

      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];

      ekf_.x_(0) = rho * cos(phi);
      ekf_.x_(1) = rho * sin(phi);

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      // Initialize state.

      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];

    }

    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    printEstimation();

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1e6;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  double dt2 = dt * dt;
  double dt3 = (dt * dt * dt) / 2.;
  double dt4 = (dt * dt * dt * dt) / 4.;

  ekf_.Q_ = MatrixXd{4, 4};

  double noise_ax = 9;
  double noise_ay = 9;

  ekf_.Q_ << dt4 * noise_ax, 0, dt3 * noise_ax, 0,
             0, dt4 * noise_ay, 0, dt3 * noise_ay,
             dt3 * noise_ax, 0, dt2 * noise_ax, 0,
             0, dt3 * noise_ay, 0, dt2 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  switch (measurement_pack.sensor_type_) {

    case MeasurementPackage::RADAR:

      Hj_ = tools.CalculateJacobian(ekf_.x_);

      ekf_.H_ = Hj_;
      ekf_.R_ = R_radar_;

      ekf_.UpdateEKF(measurement_pack.raw_measurements_);

      break;

    case MeasurementPackage::LASER:

      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;

      ekf_.Update(measurement_pack.raw_measurements_);

      break;
  }

  printEstimation();

}


void FusionEKF::printEstimation() {

  cout << "x_ = \n" << ekf_.x_ << endl;
  cout << "P_ = \n" << ekf_.P_ << endl;

}
