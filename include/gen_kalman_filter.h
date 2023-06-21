#pragma once

#include <eigen3/Eigen/Eigen>
#include "common_cpp/logger.h"

using namespace Eigen;


enum MotionModelType
{
  CONSTANT_VEL,
  CONSTANT_ACCEL,
  CONSTANT_JERK
};


class GenKalmanFilter
{

public:
  GenKalmanFilter();
  ~GenKalmanFilter();
  void init(double sigmaQ_vel, double alphaQ_vel,
    double sigmaQ_jrk, double alphaQ_jrk,
    double sigmaR_pos, double sigmaR_vel,
    double sigmaR_range, double sigmaR_zenith,
    enum MotionModelType mmtype, double dt,
    uint32_t owner_id, uint32_t id, VectorXd x);

  void predict(const double &t);
  void update(const VectorXd& measurement, bool hasVel=true);
  void update_radar(const Vector2d& measurement, const Vector2d& radarPos);
  VectorXd get_estimated_state() {return xhat_;}
  MatrixXd get_covariance_matrix() {return P_;}


  uint32_t id_;

private:
  MatrixXd build_A();
  MatrixXd build_Q();

  Eigen::MatrixXd build_C(uint32_t m_request, uint32_t n_request);
  Eigen::MatrixXd build_R(uint32_t m_request);
  Eigen::MatrixXd build_polar_R(const double& range, const double& zenith);

  void log(const double &t);

  uint32_t meas_dim_;

  // Dynamics and process noise covariance
  MatrixXd A_;
  MatrixXd C_pos_;
  MatrixXd C_vel_;
  MatrixXd Q_;
  MatrixXd R_pos_;
  MatrixXd R_vel_;

  // KF error covariance (n x n)
  MatrixXd P_;

  // Estimate state vector (n x 1)
  VectorXd xhat_;


  // State dimension (based on motion model/environment dimensionality)
  uint32_t n; // xhat = [pos, vel ...]^T

  // The timestep is used to calculate numerical derivatives
  double dt_;

  double sigmaQ_vel_ = 3;
  double alphaQ_vel_ = 0.5;
  double sigmaQ_jrk_ = 0.0075;
  double alphaQ_jrk_ = 0.5;

  // Standard deviation for the measurement covariance
  double sigmaR_pos_ = 1.0;
  double sigmaR_vel_ = 0.1;
  double sigmaR_range_ = 1.0;
  double sigmaR_zenith_ = 0.1;



  enum MotionModelType mmtype_;

  common::Logger estimate_log_;

};
