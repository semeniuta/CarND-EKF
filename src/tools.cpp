#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cerr;


VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
                              const vector<VectorXd>& ground_truth) {
  VectorXd result(4);
  result << 0,0,0,0;

  if (estimations.empty()) {
    cerr << "Empty estimations vector\n";
    return result;
  }

  if (estimations.size() != ground_truth.size()) {
    cerr << "Vector sizes mismatch\n";
    return result;
  }

  unsigned long n = estimations.size();

  for(int i = 0; i < n; i++){

    VectorXd x_hat = estimations[i];
    VectorXd x_true = ground_truth[i];

    VectorXd diff = x_hat - x_true;
    VectorXd sq_diff = diff.array() * diff.array();

    result = result + sq_diff;
  }

  result = result / n;

  result = result.array().sqrt();

  return result;

}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);

  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double psq = px * px + py * py;

  if (fabs(psq) < 1e-3) {
    cerr << "Division by zero\n";
    return Hj;
  }

  double psq_root = sqrt(psq);
  double psq_32 = psq * psq_root;

  double vx_py = vx * py;
  double vy_px = vy * px;

  double a = px / psq_root;
  double b = py / psq_root;

  Hj << a, b, 0, 0,
        -py / psq, px / psq, 0, 0,
        (py * (vx_py - vy_px)) / psq_32, (px * (vy_px - vx_py)) / psq_32, a, b;

  return Hj;

}
