#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // REVIEW: Calculate the RMSE here.

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // add up squared residual
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    // coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate the mean and squared root
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}