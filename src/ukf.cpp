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
  , nXAug{7}
  , lambda{3 - nX}
  , x{VectorXd(nX)} // initial state vector
  , P{MatrixXd::Ones(nX, nX)} // initial covariance matrix

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
  auto sigmaAug = augmentSigmaPoints();
  auto sigmaPred = predictSigmaPoints(sigmaAug, delta_t);
  predictMeanCovariance(sigmaPred);
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

Eigen::MatrixXd UKF::augmentSigmaPoints()
{
  // augment x
  auto xAug = Eigen::VectorXd(nXAug);
  xAug.head(nXAug-2) = x; 
  xAug(nXAug-2) = 0; 
  xAug(nXAug-1) = 0; // copy state to augmented vector

  // augment P
  auto PAug = Eigen::MatrixXd(nXAug, nXAug);
  PAug.fill(0.0);
  PAug.topLeftCorner(nXAug-2,nXAug-2) = P;
  PAug(nXAug-2,nXAug-2) = std_accel*std_accel;
  PAug(nXAug-1,nXAug-1) = std_yawDDot*std_yawDDot;

  // extract sigma points using augmentation
  Eigen::MatrixXd PAug_sqrt = PAug.llt().matrixL(); // square root of augmented P
  auto sigmaAug = Eigen::MatrixXd(nXAug, 2*nXAug+1); 
  sigmaAug.col(0) = xAug;
  for (int i=1; i<nXAug; ++i) {
    sigmaAug.col(i) = xAug + std::sqrt(lambda+nXAug) * PAug_sqrt.col(i);
    sigmaAug.col(i+nXAug) = xAug - std::sqrt(lambda+nXAug) * PAug_sqrt.col(i);
  }
  
  return sigmaAug;
}

Eigen::MatrixXd UKF::predictSigmaPoints(const Eigen::MatrixXd &sigmaAug, const double delta_t)
{
  // create matrix with predicted sigma points as columns
  auto Xsig_pred = Eigen::MatrixXd(nX, 2 * nXAug + 1);

  // predict sigma points
  for (int i = 0; i < 2*nXAug+1; ++i) {
    // extract values for better readability
    double p_x = sigmaAug(0,i);
    double p_y = sigmaAug(1,i);
    double v = sigmaAug(2,i);
    double yaw = sigmaAug(3,i);
    double yawd = sigmaAug(4,i);
    double nu_a = sigmaAug(5,i);
    double nu_yawdd = sigmaAug(6,i);

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
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }
  return Xsig_pred;
}

void UKF::predictMeanCovariance(const Eigen::MatrixXd &sigmaPred)
{
  Eigen::VectorXd weights = Eigen::VectorXd(2*nXAug+1);

  // set weights
  double weight_0 = lambda/(lambda+nXAug);
  weights(0) = weight_0;
  for (int i=1; i<2*nXAug+1; ++i) {  // 2n+1 weights
    double weight = 0.5/(nXAug+lambda);
    weights(i) = weight;
  }

  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * nXAug + 1; ++i) {  // iterate over sigma points
    x = x + weights(i) * sigmaPred.col(i);
  }

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * nXAug + 1; ++i) {  // iterate over sigma points
    // state difference
    Eigen::VectorXd x_diff = sigmaPred.col(i) - x;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }
}
