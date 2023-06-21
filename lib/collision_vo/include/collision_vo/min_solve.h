#ifndef MIN_SOLVE_H
#define MIN_SOLVE_H

#include <eigen3/Eigen/Eigen>
#include "ceres/ceres.h"
#include "collision_vo/utils.h"
// #include "admissible_velocities.h"

class Polynomial : public ceres::FirstOrderFunction {

public:
  Polynomial(const double& order,
    const Eigen::VectorXd& coefList,
    const std::vector<Eigen::Vector2d>& polyPoints);
  virtual ~Polynomial();

  virtual bool Evaluate(const double* parameters,
                        double* cost,
                        double* gradient) const;
  virtual int NumParameters() const { return 2; }

private:
  double order{0};
  Eigen::VectorXd coeficients;
  std::vector<Eigen::Vector2d> polyPoints;

  bool in_poly(const double& x, const double& y) const;

  double get_cost(const double& x, const double& y) const;

  double get_gradient_x(const double& x, const double& y) const;

  double get_gradient_y(const double& x, const double& y) const;


};

void minimize(const double& order,
  const Eigen::VectorXd& Coef,
  const std::vector<Eigen::Vector2d>& polyPoints,
  Eigen::Vector2d& initSolution);

#endif
