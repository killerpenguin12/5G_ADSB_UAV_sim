#ifndef UTILS_H
#define UTILS_H

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <iostream>

const double PI{3.14159265359};

bool quadratic_formula(double a, double b, double c,
  double& r1, double& r2,
  double& i1, double& i2);

double check_arcs(double input);

double get_angle(const Eigen::Vector2d& vect1, const Eigen::Vector2d& vect2);

std::vector<double> linspace(double a, double b, int n);

void switch_points(Eigen::Vector2d& one, Eigen::Vector2d& two);

int get_orientation(const Eigen::Vector2d& p1,
  const Eigen::Vector2d& p2,
  const Eigen::Vector2d& p3);

bool barycentric(const Eigen::Vector2d& A,
  const Eigen::Vector2d& B,
  const Eigen::Vector2d& C,
  const Eigen::Vector2d& P);

double distance_from_line(const Eigen::Vector2d& A,
  const Eigen::Vector2d& B,
  const Eigen::Vector2d& Pt);

double wrap_angle(double angle, double bound);

double deg_to_rad(double deg);

#endif
