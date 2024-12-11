#ifndef UKF_H
#define UKF_H

#include <tuple>

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  UKF();
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void step(MeasurementPackage meas_package);

  Eigen::VectorXd getState() const;

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void predict(double delta_t);

protected:
  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void updateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void updateRadar(MeasurementPackage meas_package);

private:
  void initialize(const MeasurementPackage& meas_package);
  Eigen::MatrixXd augmentSigmaPoints();
  Eigen::MatrixXd predictSigmaPoints(const Eigen::MatrixXd& sigmaAug, const double delta_t);
  void predictMeanCovariance(const Eigen::MatrixXd& sigmaPred); // update internal x, P

  bool isInitialized; // initially set to false, set to true in first call of ProcessMeasurement
  bool useLaser; // if this is false, laser measurements will be ignored (except for init)
  bool useRadar; // if this is false, radar measurements will be ignored (except for init)

  int nX; // State dimension
  int nX_aug; // Augmented state dimension
  int lambda; // Sigma point spreading parameter

  Eigen::VectorXd x; // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd P; // state covariance matrix 
  Eigen::VectorXd weights;  

  long long time_us; // time when the state is true, in us
  long timestampPrev;

  double std_accel; // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_yawDDot; // Process noise standard deviation yaw acceleration in rad/s^2

  double std_laserPx; // Laser measurement noise standard deviation position1 in m
  double std_laserPy; // Laser measurement noise standard deviation position2 in m

  double std_radarRange; // Radar measurement noise standard deviation radius in m
  double std_radarPhi; // Radar measurement noise standard deviation angle in rad
  double std_radarDoppler; // Radar measurement noise standard deviation radius change in m/s
};

#endif  // UKF_H