#ifndef ADMISSIBLE_H
#define ADMISSIBLE_H

#include <eigen3/Eigen/Eigen>
#include <vector>
#include "collision_vo/utils.h"

const Eigen::Vector3d e3(0, 0, 1);
const double max_thrust = 98.0665;
const Eigen::Matrix3d inertia_matrix = Eigen::Vector3d{0.6271,0.6271, 1.25}.asDiagonal();
const Eigen::Matrix3d inertia_inv = inertia_matrix.inverse();
const Eigen::Matrix3d linear_drag_matrix = Eigen::Vector3d{0.1, 0.1, 0.001}.asDiagonal();
const Eigen::Matrix3d angular_drag_matrix = Eigen::Vector3d{0.001, 0.001, 0.001}.asDiagonal();
const double g = 9.80665;
const double mass = 5;

void set_admissible_velocities(const int numPoints,
  const Eigen::VectorXd state,
  const double& dt,
  std::vector<Eigen::Vector2d>& admissibleVelocities);

void curve_fit(int order,
  const Eigen::VectorXd& x,
  const Eigen::VectorXd& y,
  const Eigen::VectorXd& z,
  Eigen::VectorXd& Coef);

void evaluate_surface(int order,
  const Eigen::VectorXd& x,
  const Eigen::VectorXd& y,
  const Eigen::VectorXd& Coef,
  Eigen::VectorXd& z);

int get_sum(int order);

#endif
