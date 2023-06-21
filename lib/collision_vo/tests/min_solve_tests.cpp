#include <gtest/gtest.h>
#include "collision_vo/min_solve.h"

TEST(CoeficientList,WhenFindingMinimumOfSurface_MinimumIsReturned)
{
  double order{3};
  int numberCoef{10};
  Eigen::VectorXd Coef(numberCoef);
  Coef << -0.68517408,  0.31619721,  0.41684618, -0.04772924, -0.18273136,
           0.05650831, -0.00537944, -0.08074328,  0.01440997, -0.0195281;

  double x{0.2};
  double y{0.1};

  Eigen::Vector2d init{x,y};

  std::vector<Eigen::Vector2d> points = {Eigen::Vector2d{-3, 3}, Eigen::Vector2d{3, 3},
      Eigen::Vector2d{3, -3}, Eigen::Vector2d{-3, -3}};


  minimize(order,Coef,points,init);

  double truth[] = {-2.67591979, -2.07831818};
  // [-2.67591979 -2.07831818]

  EXPECT_NEAR(init[0], truth[0], 1e-5);
  EXPECT_NEAR(init[1], truth[1], 1e-5);

}
