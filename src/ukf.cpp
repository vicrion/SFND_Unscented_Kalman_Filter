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
  , lambda{3 - nX_aug}
  , x{VectorXd(nX)} // initial state vector
  , P{MatrixXd::Ones(nX, nX)} // initial covariance matrix
  , Xsigma_pred{Eigen::MatrixXd(nX, 2 * nX_aug + 1)}
  , weights{Eigen::VectorXd(2*nX_aug+1)} // const

  , timestampPrev{0}

  , std_accel{10} // Process noise standard deviation longitudinal acceleration in m/s^2, TODO: optimize
  , std_yawDDot{0.1} // Process noise standard deviation yaw acceleration in rad/s^2, TODO: optimize
  , std_laserPx{0.15} // const, Laser measurement noise standard deviation position1 in m
  , std_laserPy{0.15} // const, Laser measurement noise standard deviation position2 in m
  , std_radarRange{0.3} // const, Radar measurement noise standard deviation radius in m
  , std_radarPhi{0.03} // const, Radar measurement noise standard deviation angle in rad
  , std_radarDoppler{0.3} // const, Radar measurement noise standard deviation radius change in m/s
{
  // P.diagonal() << 1, 1, 1, 1, 1; // std_accel*std_accel, std_yawDDot*std_yawDDot

  // set weights
  double weight_0 = lambda/(lambda+nX_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*nX_aug+1; ++i) {  // 2n+1 weights
    double weight = 0.5/(nX_aug+lambda);
    weights(i) = weight;
  }
}

UKF::~UKF() {}

void UKF::step(MeasurementPackage meas_package) 
{
  // UKF: initialize x, P using package data and std values
  if (!isInitialized){
    initialize(meas_package);
    return;
  }

  auto deltaT = static_cast<double>((meas_package.timestamp_ - timestampPrev) / 1e6);
  timestampPrev = meas_package.timestamp_;

  // UKF: predict and update
  predict(deltaT);
  switch (meas_package.sensor_type_)
  {
  case MeasurementPackage::LASER:
  {
    updateLidar(meas_package);
    break;
  }
  case MeasurementPackage::RADAR:
  {
    updateRadar(meas_package);
    break;
  }
  default:
  {
    std::cerr << "Unknown package type, cannot run update step.\n";
    break;
  }
  }
}

Eigen::VectorXd UKF::getState() const
{
    return x;
}

void UKF::predict(double delta_t) 
{
  // predict: x and P using augmentation
  auto Xsigma_aug = augmentSigmaPoints();
  predictSigmaPoints(Xsigma_aug, delta_t);
  predictMeanCovariance();
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
  int n_z = meas_package.raw_measurements_.size(); // 3

  // transform sigma into radar coordinate system
  // calculate mean of transformed points
  // calculate measurement covariance S
}

Eigen::MatrixXd UKF::augmentSigmaPoints()
{
  // augment x
  auto x_aug = Eigen::VectorXd(nX_aug);
  x_aug.head(nX_aug-2) = x; 
  x_aug(nX_aug-2) = 0; 
  x_aug(nX_aug-1) = 0; // copy state to augmented vector

  // augment P
  auto P_aug = Eigen::MatrixXd(nX_aug, nX_aug);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(nX_aug-2,nX_aug-2) = P;
  P_aug(nX_aug-2,nX_aug-2) = std_accel*std_accel;
  P_aug(nX_aug-1,nX_aug-1) = std_yawDDot*std_yawDDot;

  // extract sigma points using augmentation
  Eigen::MatrixXd P_aug_sqrt = P_aug.llt().matrixL(); // square root of augmented P
  auto Xsigma_aug = Eigen::MatrixXd(nX_aug, 2*nX_aug+1); 
  Xsigma_aug.col(0) = x_aug;
  for (int i=1; i<nX_aug; ++i) {
    Xsigma_aug.col(i) = x_aug + std::sqrt(lambda+nX_aug) * P_aug_sqrt.col(i);
    Xsigma_aug.col(i+nX_aug) = x_aug - std::sqrt(lambda+nX_aug) * P_aug_sqrt.col(i);
  }
  
  return Xsigma_aug;
}

void UKF::initialize(const MeasurementPackage& meas_package)
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

    P(0, 0) = std_laserPx * std_laserPx;
    P(1, 1) = std_laserPy * std_laserPy;
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
}

void UKF::predictSigmaPoints(const Eigen::MatrixXd &Xsigma_aug, const double delta_t)
{
  // predict sigma points
  for (int i = 0; i < 2*nX_aug+1; ++i) {
    // extract values for better readability
    double p_x = Xsigma_aug(0,i);
    double p_y = Xsigma_aug(1,i);
    double v = Xsigma_aug(2,i);
    double yaw = Xsigma_aug(3,i);
    double yawd = Xsigma_aug(4,i);
    double nu_a = Xsigma_aug(5,i);
    double nu_yawdd = Xsigma_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (std::fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( std::sin(yaw + yawd*delta_t) - std::sin(yaw));
        py_p = p_y + v/yawd * ( std::cos(yaw) - std::cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*std::cos(yaw);
        py_p = p_y + v*delta_t*std::sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * std::cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * std::sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsigma_pred(0,i) = px_p;
    Xsigma_pred(1,i) = py_p;
    Xsigma_pred(2,i) = v_p;
    Xsigma_pred(3,i) = yaw_p;
    Xsigma_pred(4,i) = yawd_p;
  }
}

void UKF::predictMeanCovariance()
{
  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * nX_aug + 1; ++i) {  // iterate over sigma points
    x = x + weights(i) * Xsigma_pred.col(i);
  }

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * nX_aug + 1; ++i) {  // iterate over sigma points
    // state difference
    Eigen::VectorXd x_diff = Xsigma_pred.col(i) - x;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }
}
