#include "ukf.h"
#include <iostream>
#include <assert.h>

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() 
  : is_initialized_{false}
  , use_laser_{true}
  , use_radar_{true}
  , timestampPrev{0}
  , std_a_{30} // Process noise standard deviation longitudinal acceleration in m/s^2, TODO: optimize
  , std_yawdd_{0.1} // Process noise standard deviation yaw acceleration in rad/s^2, TODO: optimize
  , std_laspx_{0.15} // const, Laser measurement noise standard deviation position1 in m
  , std_laspy_{0.15} // const, Laser measurement noise standard deviation position2 in m
  , std_radr_{0.3} // const, Radar measurement noise standard deviation radius in m
  , std_radphi_{0.03} // const, Radar measurement noise standard deviation angle in rad
  , std_radrd_{0.3} // const, Radar measurement noise standard deviation radius change in m/s
  , n_x_{5}
  , n_aug_{7}
  , lambda_{3 - n_x_}
{
  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd::Zero(n_x_, n_x_);
  P_.diagonal() << 1, 1, 1, 1, 1; // std_a_*std_a_, std_yawdd_*std_yawdd_

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_x_ + 1); // 5x11

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
  if (!is_initialized_)
  {
    switch (meas_package.sensor_type_)
    {
      // initialize state's position using direct measurements
      // leave unitialized the rest (0): velocity, yaw angle, yaw rate
    case MeasurementPackage::LASER:
    {
      std::cout << "State will be initialized using LIDAR measurement.\n";
      assert(meas_package.raw_measurements_.size() == 2);
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);

      P_(0,0) = std_laspx_*std_laspx_;
      P_(1,1) = std_laspy_*std_laspy_;
      break;
    }

    case MeasurementPackage::RADAR:
    {
      std::cout << "State will be initialized using RADAR measurement.\n";
      assert(meas_package.raw_measurements_.size() == 3);
      auto range = meas_package.raw_measurements_(0);
      auto phi = meas_package.raw_measurements_(1);
      auto rhoDot = meas_package.raw_measurements_(2);
      x_(0) = range * std::cos(phi);
      x_(1) = range * std::sin(phi);
      break;
    } 
    default:
      break;
    }

    is_initialized_ = true;
    timestampPrev = meas_package.timestamp_;
    return;
  }

  auto deltaT = static_cast<double>((meas_package.timestamp_ - timestampPrev) / 1e6);
  timestampPrev = meas_package.timestamp_;

  Prediction(deltaT);

  if (meas_package.LASER){
    UpdateLidar(meas_package);
  }
  else {
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // Belief about object's position: modify state vector and covariance matrix

  // calculate lidar NIS
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}