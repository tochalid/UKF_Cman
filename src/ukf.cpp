#include "ukf.h"
#include <iomanip>
#include <iostream>
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // REVIEW: set state dimension
  n_x_ = 5;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;

  // DO NOT MODIFY measurement noise values below these are provided by the
  // sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  // DO NOT MODIFY measurement noise values above these are provided by the
  // sensor manufacturer.

  // REVIEW: set augmented dimension
  n_aug_ = n_x_ + 2;

  // define spreading parameter
  lambda_ = 3 - n_aug_;

  // set false to enable init with first measurements
  is_initialized_ = false;

  // create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0,
      0, std_radrd_ * std_radrd_;

  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;

  // Open file and write header for NIS statistics
  NIS_data_file_.open("NIS_data.csv", ios::out);
  NIS_data_file_ << MeasurementPackage::RADAR << " - RADAR,"
                 << MeasurementPackage::LASER << " - LIDAR,"
                 << "R:X².050,"
                 << "L:X².050," << std::endl;
}

UKF::~UKF() { NIS_data_file_.close(); }

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // REVIEW:
  if (!is_initialized_) {
    P_ << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 1;

    // first measurements
    // convert radar from polar to cartesian and initialize state
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = (double)meas_package.raw_measurements_[0];
      double phi = (double)meas_package.raw_measurements_[1];
      double rho_dot = (double)meas_package.raw_measurements_[2];

      double p_x = rho * cos(phi);
      double p_y = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      double v = sqrt(vx * vx + vy * vy);

      x_ << p_x, p_y, v, 0, 0;

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1], 0, 0, 0;
    }

    time_us_ = meas_package.timestamp_;

    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    for (int i = 1; i < (2 * n_aug_ + 1); i++) {
      double weight_ = 0.5 / (n_aug_ + lambda_);
      weights_(i) = weight_;
    }

    // done init, no predict and update required
    is_initialized_ = true;

  } else {
    // predict and update current state
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;

    time_us_ = meas_package.timestamp_;

    Prediction(delta_t);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      n_z_ = 3;
      UpdateRadar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER &&
               use_laser_) {
      n_z_ = 2;
      UpdateLidar(meas_package);
    }
  }
}

// Helper function normalizing angle
inline void NormalizeAngle(double *angle) {
  while (*angle > M_PI) *angle -= 2. * M_PI;
  while (*angle < -M_PI) *angle += 2. * M_PI;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  REVIEW:
  Estimates the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance
  matrix.
  */

  // AUGMENTED SIGMA POINTS

  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(n_aug_ - 2) = 0;
  x_aug(n_aug_ - 1) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_aug_ - 1, n_aug_ - 1) = std_yawdd_ * std_yawdd_;
  P_aug(n_aug_ - 2, n_aug_ - 2) = std_a_ * std_a_;

  // create square root matrix
  MatrixXd A_aug = MatrixXd(7, 7);
  A_aug = P_aug.llt().matrixL();
  // create augmented sigma points
  Xsig_aug_.col(0) = x_aug;

  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug + (sqrt(lambda_ + n_aug_) * A_aug.col(i));
    Xsig_aug_.col(n_aug_ + i + 1) =
        x_aug - (sqrt(lambda_ + n_aug_) * A_aug.col(i));
  }

  // SIGMA POINT PREDICTION OF THE STATE

  // create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // predict sigma points
  for (int i = 0; i < ((2 * n_aug_) + 1); i++) {
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + (v / yawd * (sin(yaw + (yawd * delta_t)) - sin(yaw)));
      py_p = p_y + (v / yawd * (cos(yaw) - cos(yaw + (yawd * delta_t))));
    } else {
      px_p = p_x + (v * delta_t * cos(yaw));
      py_p = p_y + (v * delta_t * sin(yaw));
    }

    double v_p = v;
    double yaw_p = yaw + (yawd * delta_t);
    double yawd_p = yawd;

    // add noise
    px_p = px_p + (0.5 * nu_a * delta_t * delta_t * cos(yaw));
    py_p = py_p + (0.5 * nu_a * delta_t * delta_t * sin(yaw));
    v_p = v_p + (nu_a * delta_t);

    yaw_p = yaw_p + (0.5 * nu_yawdd * delta_t * delta_t);
    yawd_p = yawd_p + (nu_yawdd * delta_t);

    // write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // PREDICT MEAN AND COVARIANCE

  // predict state mean
  x_.fill(0.0);
  for (int i = 0; i < ((2 * n_aug_) + 1); i++) {
    x_ = x_ + (weights_(i) * Xsig_pred_.col(i));
  }

  // predict state covariance matrix
  P_.fill(0.0);
  // iterate over sigma points
  for (int i = 0; i < ((2 * n_aug_) + 1); i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // normalize
    NormalizeAngle(&x_diff(3));
    P_ = P_ + weights_(i) * ((x_diff) * (x_diff).transpose());
  }
}

/**
 * Updates the state and the state covariance matrix using a laser
 * measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // REVIEW:
  // Uses lidar data to update the belief about the object's position.
  // Modify the state vector, x_, and covariance, P_.

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1;
       i++) {  // 2n+1 simga points
               // extract values for better readibility
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  UpdateUKF(meas_package, Zsig);  // also calculates NIS
}

/**
 * Updates the state and the state covariance matrix using a radar
 * measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // REVIEW:
  // Uses radar data to update the belief about the object's position.
  // Modify the state vector, x_, and covariance, P_.

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);         // r
    Zsig(1, i) = atan2(p_y, p_x);                     // phi
    Zsig(2, i) = (p_x * v1 + p_y * v2) / Zsig(0, i);  // r_dot
  }

  UpdateUKF(meas_package, Zsig);  // also calculates NIS
}

// REVIEW:
void UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd &Zsig) {
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);

  z_pred.fill(0.0);

  z_pred = Zsig * weights_;

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);

  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      NormalizeAngle(&z_diff(1));
    }

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    S = S + R_radar_;
  } else {
    S = S + R_lidar_;
  }

  // create matrix for cross correlation T
  MatrixXd T = MatrixXd(n_x_, n_z_);

  T.fill(0.0);

  for (int i = 0; i < ((2 * n_aug_) + 1); i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      NormalizeAngle(&z_diff(1));
    }

    NormalizeAngle(&x_diff(3));

    T = T + weights_(i) * x_diff * (z_diff.transpose());
  }

  MatrixXd K = MatrixXd(n_x_, n_z_);

  // calculate Kalman gain K
  K = T * S.inverse();

  VectorXd z = meas_package.raw_measurements_;

  // update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    NormalizeAngle(&z_diff(1));
  }

  x_ = x_ + (K * z_diff);

  P_ = P_ - (K * S * K.transpose());

  // CALCULATE AND PRINT NIS: NORMALIZED INNOVATION SQUARED

  // write NIS Info in two colums
  NIS_ = z_diff.transpose() * S.inverse() * z_diff;
  NIS_data_file_ << NIS_ << ",";
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    NIS_data_file_ << "7.815,5.991" << std::endl;
  }
}
