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
  : isInitialized{false}
  , useLaser{true}
  , useRadar{true}
  
  , nX{5}
  , nX_aug{7}
  , lambda{3 - nX}

  , timestampPrev{0}
  , std_accel{10} // Process noise standard deviation longitudinal acceleration in m/s^2, TODO: optimize
  , std_yawDDot{0.1} // Process noise standard deviation yaw acceleration in rad/s^2, TODO: optimize
  , std_laserPx{0.15} // const, Laser measurement noise standard deviation position1 in m
  , std_laserPy{0.15} // const, Laser measurement noise standard deviation position2 in m
  , std_radarRange{0.3} // const, Radar measurement noise standard deviation radius in m
  , std_radarPhi{0.03} // const, Radar measurement noise standard deviation angle in rad
  , std_radarDoppler{0.3} // const, Radar measurement noise standard deviation radius change in m/s
  
{
  // initial state vector
  x = VectorXd(nX);

  // initial covariance matrix
  P = MatrixXd::Zero(nX, nX);
  P.diagonal() << 1, 1, 1, 1, 1; // std_accel*std_accel, std_yawDDot*std_yawDDot

  XSigPred = MatrixXd(nX, 2 * nX + 1); // 5x11

}

UKF::~UKF() {}

void UKF::step(MeasurementPackage meas_package) 
{
  if (!isInitialized)
  {
    switch (meas_package.sensor_type_)
    {
      // initialize state's position using direct measurements
      // leave unitialized the rest (0): velocity, yaw angle, yaw rate
    case MeasurementPackage::LASER:
    {
      std::cout << "State will be initialized using LIDAR measurement.\n";
      assert(meas_package.raw_measurements_.size() == 2);
      x(0) = meas_package.raw_measurements_(0);
      x(1) = meas_package.raw_measurements_(1);

      P(0,0) = std_laserPx*std_laserPx;
      P(1,1) = std_laserPy*std_laserPy;
      break;
    }

    case MeasurementPackage::RADAR:
    {
      std::cout << "State will be initialized using RADAR measurement.\n";
      assert(meas_package.raw_measurements_.size() == 3);
      auto range = meas_package.raw_measurements_(0);
      auto phi = meas_package.raw_measurements_(1);
      auto rhoDot = meas_package.raw_measurements_(2);
      x(0) = range * std::cos(phi);
      x(1) = range * std::sin(phi);
      break;
    } 
    default:
      break;
    }

    isInitialized = true;
    timestampPrev = meas_package.timestamp_;
    return;
  }

  auto deltaT = static_cast<double>((meas_package.timestamp_ - timestampPrev) / 1e6);
  timestampPrev = meas_package.timestamp_;

  predict(deltaT);

  if (meas_package.LASER){
    updateLidar(meas_package);
  }
  else {
    updateRadar(meas_package);
  }
}

Eigen::VectorXd UKF::getState() const
{
    return x;
}

void UKF::predict(double delta_t) 
{
  std::cout << "Predict.\n";
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
}

void UKF::updateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // Belief about object's position: modify state vector and covariance matrix

  // calculate lidar NIS
}

void UKF::updateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}