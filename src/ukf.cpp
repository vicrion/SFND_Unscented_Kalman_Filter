#include "ukf.h"
#include <iostream>
#include <assert.h>

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(bool useLidar, bool useRadar, bool debug) 
  : isInitialized{false}
  , useLaser{useLidar}
  , useRadar{useRadar}
  , debug{debug}
  
  , nX{5}
  , nX_aug{7}
  , lambda{3 - nX_aug}

  , x{VectorXd::Zero(nX)} // initial state vector
  , P{MatrixXd::Zero(nX, nX)} // initial covariance matrix
  , Xsigma_pred{Eigen::MatrixXd::Zero(nX, 2 * nX_aug + 1)}
  , weights{Eigen::VectorXd::Zero(2*nX_aug+1)} // const

  , timestampPrev{0}

  , std_accel{10} // Process noise standard deviation longitudinal acceleration in m/s^2, TODO: optimize
  , std_yawDDot{1.5} // Process noise standard deviation yaw acceleration in rad/s^2, TODO: optimize
  
  , std_laserPx{0.15} // const, Laser measurement noise standard deviation position1 in m
  , std_laserPy{0.15} // const, Laser measurement noise standard deviation position2 in m
  , std_radarRange{0.3} // const, Radar measurement noise standard deviation radius in m
  , std_radarPhi{0.03} // const, Radar measurement noise standard deviation angle in rad
  , std_radarDoppler{0.3} // const, Radar measurement noise standard deviation radius change in m/s
  , nIter{0}
{
  // state uncertainties: x, y, vel, yaw, yaw_rate
  P.diagonal() << 0.5, 0.5, 0.5, 0.005, 0.05;

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
  nIter++;
  // UKF: initialize x, P using package data and std values
  if (!isInitialized){
    initialize(meas_package);
    return;
  }

  auto deltaT = static_cast<double>((meas_package.timestamp_ - timestampPrev) / 1e6);
  timestampPrev = meas_package.timestamp_;

  // UKF: predict and update
  switch (meas_package.sensor_type_)
  {
  case MeasurementPackage::LASER:
  {
    if (useLaser){
      predict(deltaT);
      updateLidar(meas_package);
    }
    break;
  }
  case MeasurementPackage::RADAR:
  {
    if (useRadar){
      predict(deltaT);
      updateRadar(meas_package);
    }
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

  if (debug){
    std::cout << nIter << ": predicted x = [" << x.transpose() << "].\n";
  }
}

void UKF::updateLidar(MeasurementPackage meas_package) 
{
  // Belief about object's position: modify state vector and covariance matrix
  int n_z = meas_package.raw_measurements_.size(); // 2
  auto z = meas_package.raw_measurements_;
  Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * nX_aug + 1);
  for (int i = 0; i < 2 * nX_aug + 1; ++i)
  {
    Zsig(0, i) = Xsigma_pred(0, i); // px
    Zsig(1, i) = Xsigma_pred(1, i); // py
  }
  
  Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * nX_aug + 1; ++i)
  {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * nX_aug + 1; ++i)
  {
    // Residual
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  Eigen::MatrixXd R = Eigen::MatrixXd(n_z, n_z);
  R << std_laserPx * std_laserPx, 0,
      0, std_laserPy * std_laserPy;
  S = S + R;

  // Cross-correlation matrix Tc
  Eigen::MatrixXd Tc = Eigen::MatrixXd(nX, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * nX_aug + 1; ++i)
  {
    // Residual
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;

    // State difference
    Eigen::VectorXd x_diff = Xsigma_pred.col(i) - x;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K
  Eigen::MatrixXd K = Tc * S.inverse();

  // Residual
  Eigen::VectorXd z_diff = z - z_pred;

  // Update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K * S * K.transpose();

  if (debug){
    std::cout << "P=\n" << P << std::endl;

    double NIS = z_diff.transpose() * S.inverse() * z_diff;
     std::cout << nIter  << ": upd-lidar x = [" << x.transpose() << "], " << 
      "meas=[" << z.transpose() << "], NIS=" << NIS << ".\n";
  }
}

void UKF::updateRadar(MeasurementPackage meas_package) 
{
  int n_z = meas_package.raw_measurements_.size(); // 3

  // transform sigma points into measurement (radar) space
  MatrixXd Zsig = MatrixXd(n_z, 2 * nX_aug + 1); // matrix for sigma points in measurement space
  VectorXd z_pred = VectorXd(n_z); // mean predicted measurement
  for (int i = 0; i < 2 * nX_aug + 1; ++i) {  // 2n+1 sigma points
    double p_x = Xsigma_pred(0,i);
    double p_y = Xsigma_pred(1,i);
    double v  = Xsigma_pred(2,i);
    double yaw = Xsigma_pred(3,i);

    double vx = cos(yaw)*v;
    double vy = sin(yaw)*v;

    // measurement (radar) model
    auto dist = sqrt(p_x*p_x + p_y*p_y);
    Zsig(0,i) = dist;             // r
    Zsig(1,i) = atan2(p_y, p_x);  // phi
    if (std::fabs(dist) > 1e-3)
      Zsig(2, i) = (p_x*vx + p_y*vy) / dist; // r_dot
    else
      Zsig(2, i) = 0;
  }

  // mean predicted measurement (of transformed points)
  z_pred.fill(0.0);
  for (int i=0; i < 2*nX_aug+1; ++i) {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }
  z_pred(1) = wraptopi(z_pred(1));

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z); // measurement covariance
  S.fill(0.0);
  for (int i = 0; i < 2 * nX_aug + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    z_diff(1) = wraptopi(z_diff(1));
    
    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radarRange*std_radarRange, 0, 0,
        0, std_radarPhi*std_radarPhi, 0,
        0, 0,std_radarDoppler*std_radarDoppler;
  S = S + R;

  // UKF update: calc cross-correlation, Kalman gain, x and P
  auto z = meas_package.raw_measurements_;
  MatrixXd Tc = MatrixXd(nX, n_z); // matrix for cross correlation Tc
  Tc.fill(0.0);
  for (int i = 0; i < 2 * nX_aug + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    z_diff(1) = wraptopi(z_diff(1));

    // state difference
    VectorXd x_diff = Xsigma_pred.col(i) - x;
    // angle normalization
    x_diff(3) = wraptopi(x_diff(3));

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }
  MatrixXd K = Tc * S.inverse(); // Kalman gain
  VectorXd z_diff = z - z_pred; // residual

  // angle normalization
  z_diff(1) = wraptopi(z_diff(1));
  
  x = x + K * z_diff;
  P = P - K*S*K.transpose();

  if (debug){
    double NIS = z_diff.transpose() * S.inverse() * z_diff; // calculate radar NIS
     std::cout << nIter  << ": upd-radar x = [" << x.transpose() << 
      "], meas=[" << z[0]*std::cos(z[1]) << ", " << z[0]*std::sin(z[1]) << ", " << z[2] << 
      "], polar=[" << z[0] << ", " << z[1] << "], NIS=" << NIS << ".\n";
  }
}

Eigen::MatrixXd UKF::augmentSigmaPoints()
{
  // augment x
  auto x_aug = Eigen::VectorXd(nX_aug);
  x_aug.head(nX_aug-2) = x; 
  x_aug(nX_aug-2) = 0; 
  x_aug(nX_aug-1) = 0; // copy state to augmented vector

  // augment P with matrix Q
  auto P_aug = Eigen::MatrixXd(nX_aug, nX_aug);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(nX_aug-2,nX_aug-2) = P;
  // matrix Q - process noise
  P_aug(nX_aug-2,nX_aug-2) = std_accel*std_accel;
  P_aug(nX_aug-1,nX_aug-1) = std_yawDDot*std_yawDDot;

  // extract sigma points using augmentation
  Eigen::MatrixXd P_aug_sqrt = P_aug.llt().matrixL(); // square root of augmented P
  auto Xsigma_aug = Eigen::MatrixXd(nX_aug, 2*nX_aug+1); 
  Xsigma_aug.col(0) = x_aug;
  const auto mult = std::sqrt(lambda+nX_aug);
  for (int i=0; i<nX_aug; ++i) {
    Xsigma_aug.col(i+1) = x_aug + mult * P_aug_sqrt.col(i);
    Xsigma_aug.col(i+1+nX_aug) = x_aug - mult * P_aug_sqrt.col(i);
  }

  return Xsigma_aug;
}

void UKF::initialize(const MeasurementPackage& meas_package)
{
  switch (meas_package.sensor_type_)
  {
  case MeasurementPackage::LASER:
  {
    if (!useLaser) return;

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
    if (!useRadar) return;

    std::cout << "State will be initialized using RADAR measurement.\n";
    assert(meas_package.raw_measurements_.size() == 3);
    auto r = meas_package.raw_measurements_(0);
    auto phi = meas_package.raw_measurements_(1);
    auto rDot = meas_package.raw_measurements_(2);
    x(0) = r * std::cos(phi);
    x(1) = r * std::sin(phi);
    break;
  }
  default: 
  {
    std::cerr << "Failed to initialize: unkown sensor type.\n";
    isInitialized = false;
    return;
  }
  }
  // just some random values for now, so they are not zeros
  x(2) = 1;
  x(3) = 0.1;
  x(4) = 0;

  isInitialized = true;
  timestampPrev = meas_package.timestamp_;

  if (debug){
    std::cout << nIter  << ": initialized x = [" << x.transpose() << "], using meas=[" << meas_package.raw_measurements_.transpose() << "].\n";
    std::cout << "initialized P=\n" << P << std::endl;
  }
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
    if (std::fabs(yawd) > 1e-3) {
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
    x_diff(3) = wraptopi(x_diff(3));

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }
}

double UKF::wraptopi(const double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}
