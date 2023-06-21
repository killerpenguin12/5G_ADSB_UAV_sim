#include <gtest/gtest.h>
// #include <math>
#include "collision_vo/utils.h"
// #include "src/utils.cpp"

//quadratic formula
TEST(ABCValuesForQuadraticEqn, WhenSolving_TheValuesAreCorrect)
{
  double a = 90;
  double b = 162;
  double c = 45;

  double x1;
  double x2;
  double i1;
  double i2;

  quadratic_formula(a, b, c, x1, x2, i1, i2);

  EXPECT_EQ(x1, -9/10.0 + 1/10.0 * sqrt(31));
  EXPECT_EQ(x2, -9/10.0 - 1/10.0 * sqrt(31));
  EXPECT_EQ(i1, 0);
  EXPECT_EQ(i2, 0);
}

//check_arcs

//get_angle
TEST(TwoAngles, WhenFindingAngle_AngleIsCorrect)
{
  double PI = 3.14159265359;

  Eigen::Vector2d A(0,1);
  Eigen::Vector2d B(0,-1);

  double angle = get_angle(A,B);
  EXPECT_NEAR(angle,-PI,1e-8);

  A = Eigen::Vector2d(0,1);
  B = Eigen::Vector2d(1,0);
  angle = get_angle(A,B);
  EXPECT_NEAR(angle,-PI/2,1e-8);

  A = Eigen::Vector2d(1,0);
  B = Eigen::Vector2d(0,1);
  angle = get_angle(A,B);
  EXPECT_NEAR(angle,PI/2,1e-8);

  A = Eigen::Vector2d(1,0);
  B = Eigen::Vector2d(-1,0);
  angle = get_angle(A,B);
  EXPECT_NEAR(angle,PI,1e-8);

  A = Eigen::Vector2d(1,0);
  B = Eigen::Vector2d(0,-1);
  angle = get_angle(A,B);
  EXPECT_NEAR(angle,-PI/2,1e-8);

  A = Eigen::Vector2d(1,0);
  B = Eigen::Vector2d(1,1);
  angle = get_angle(A,B);
  EXPECT_NEAR(angle,PI/4,1e-8);
}

//linspace
TEST(StartingAndEndingAndNumberOfValues, WhenLinspacing_ValuesAreCorrect)
{
  double start = -1.0;
  double end = 1.0;
  int numberOfValues = 11;

  std::vector<double> values = linspace(start, end, numberOfValues);

  for(int i(0); i<values.size(); i++)
  {
    EXPECT_NEAR(values[i],start+i*0.2,1e-8);
  }
}

//Switching Points
TEST(TwoPoints, WhenSwitchingTheirOrder_TheyAreSwitched)
{
  Eigen::Vector2d A(-2,-6);
  Eigen::Vector2d B(2,7);

  switch_points(A,B);

  EXPECT_EQ(A[0],2);
  EXPECT_EQ(A[1],7);
  EXPECT_EQ(B[0],-2);
  EXPECT_EQ(B[1],-6);
}

// Orientation
TEST(ThreeCounterClockwisePoints, WhenCalculatingOrientation_FunctionReturnsTwo)
{
  Eigen::Vector2d A(-2,-6);
  Eigen::Vector2d B(2,-6);
  Eigen::Vector2d C(1,2);

  int orientation = get_orientation(A,B,C);

  EXPECT_EQ(orientation,2);
}

// TEST(ThreeCounterClockwisePointsTest, WhenCalculatingOrientation_FunctionReturnsTwo)
// {
//   Eigen::Vector2d A(0.543647,0.932479);
//   Eigen::Vector2d B(1.28178,0.9);
//   Eigen::Vector2d C(1.0,1.0);
//
//   int orientation = get_orientation(A,B,C);
//
//   EXPECT_EQ(orientation,2);
// }

//Barycentric test
TEST(APointAndACollisionCone, WhenCaculatingBarycentric_TheBoolsAreCorrect)
{
  Eigen::Vector2d A(0,0);
  Eigen::Vector2d B(1,3);
  Eigen::Vector2d C(2,-8);
  Eigen::Vector2d Pt(1,0);

  EXPECT_TRUE(barycentric(A,B,C,Pt));

  Pt = Eigen::Vector2d(-1,1);
  EXPECT_FALSE(barycentric(A,B,C,Pt));
}

//Distance from Line
TEST(APointAndTwoPointsOfALine, WhenDistanceFromPointToLine_TheDistanceIsCorrect)
{
  Eigen::Vector2d A(-1,0);
  Eigen::Vector2d B(1,0);
  Eigen::Vector2d Pt(0,1);

  double distance = distance_from_line(A,B,Pt);

  EXPECT_EQ(distance, 1.0);

  A = Eigen::Vector2d(-1,-1);
  B = Eigen::Vector2d(1,1);
  Pt = Eigen::Vector2d(-1,1);

  distance = distance_from_line(A,B,Pt);

  EXPECT_NEAR(distance, sqrt(2.0), 1e-8);

  A = Eigen::Vector2d(0,-5);
  B = Eigen::Vector2d(0,5);;
  Pt = Eigen::Vector2d(-1,1);

  distance = distance_from_line(A,B,Pt);

  EXPECT_EQ(distance, 1);
}
