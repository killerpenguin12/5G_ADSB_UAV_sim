#include <gtest/gtest.h>
#include <vector>
#include "collision_vo/collision_vo.h"


TEST(PositionAndVelocityOfTwoVehicles, WhenCalculatingVOCone_ConeIsCorrectlyCalculated)
{
  Eigen::Vector2d av1Pos(0,0);
  Eigen::Vector2d av1Vel(1,1);
  Eigen::Vector2d av2Pos(6.5,3);
  Eigen::Vector2d av2Vel(-1,0);

  Eigen::Vector2d posUncertainty(0,0);

  Eigen::Vector2d avR = av2Pos - av1Pos;

  CollisionVO cvo = CollisionVO();
  std::vector<Eigen::Vector2d> output;
  cvo.get_collision_cone(av1Pos, av1Vel, av2Pos, av2Vel, posUncertainty, output);

  // ASSERT_TRUE();

  // Eigen::Vector2d finalA = av1Pos +

  Eigen::Vector2d A = output[0];
  Eigen::Vector2d B = output[1];
  Eigen::Vector2d C = output[2];

  EXPECT_NEAR(A[0], (av1Pos + (av1Vel + av2Vel) / 2)[0], 1e-8);
  EXPECT_NEAR(A[1], (av1Pos + (av1Vel + av2Vel) / 2)[1], 1e-8);

  Eigen::Vector2d AB = B - A;
  Eigen::Rotation2D<double> rotateB(PI/2);
  Eigen::Vector2d BTanPoint = avR + (rotateB*AB).normalized()*2*cvo.collisionRadius;

  EXPECT_NEAR(std::sqrt(std::pow(AB[0],2)+std::pow(AB[1],2)), cvo.range, 1e-8);
  EXPECT_NEAR(BTanPoint.normalized()[0], AB.normalized()[0], 1e-8);
  EXPECT_NEAR(BTanPoint.normalized()[1], AB.normalized()[1], 1e-8);

  Eigen::Vector2d AC = C - A;
  Eigen::Rotation2D<double> rotateC(-PI/2);
  Eigen::Vector2d CTanPoint = avR + (rotateC*AC).normalized()*2*cvo.collisionRadius;

  EXPECT_NEAR(std::sqrt(std::pow(AC[0],2)+std::pow(AC[1],2)), cvo.range, 1e-8);
  EXPECT_NEAR(CTanPoint.normalized()[0], AC.normalized()[0], 1e-8);
  EXPECT_NEAR(CTanPoint.normalized()[1], AC.normalized()[1], 1e-8);

}

TEST(PositionAndVelocityOfTwoVehicles2, WhenCalculatingVOCone_ConeIsCorrectlyCalculated)
{
  Eigen::Vector2d av1Pos(1, 0.25);
  Eigen::Vector2d av1Vel(1, 1);
  Eigen::Vector2d av2Pos(-9.5, 3);
  Eigen::Vector2d av2Vel(1, -3);

  Eigen::Vector2d posUncertainty(0,0);

  Eigen::Vector2d avR = av2Pos - av1Pos;

  CollisionVO cvo = CollisionVO();
  std::vector<Eigen::Vector2d> output;
  cvo.get_collision_cone(av1Pos, av1Vel, av2Pos, av2Vel, posUncertainty, output);

  // ASSERT_TRUE();

  // Eigen::Vector2d finalA = av1Pos +

  Eigen::Vector2d A = output[0];
  Eigen::Vector2d B = output[1];
  Eigen::Vector2d C = output[2];

  EXPECT_NEAR(A[0], (av1Pos + (av1Vel + av2Vel) / 2)[0], 1e-8);
  EXPECT_NEAR(A[1], (av1Pos + (av1Vel + av2Vel) / 2)[1], 1e-8);

  Eigen::Vector2d AB = B - A;
  Eigen::Rotation2D<double> rotateB(PI/2);
  Eigen::Vector2d BTanPoint = avR + (rotateB*AB).normalized()*2*cvo.collisionRadius;

  EXPECT_NEAR(std::sqrt(std::pow(AB[0],2)+std::pow(AB[1],2)), cvo.range, 1e-8);
  EXPECT_NEAR(BTanPoint.normalized()[0], AB.normalized()[0], 1e-8);
  EXPECT_NEAR(BTanPoint.normalized()[1], AB.normalized()[1], 1e-8);

  Eigen::Vector2d AC = C - A;
  Eigen::Rotation2D<double> rotateC(-PI/2);
  Eigen::Vector2d CTanPoint = avR + (rotateC*AC).normalized()*2*cvo.collisionRadius;

  EXPECT_NEAR(std::sqrt(std::pow(AC[0],2)+std::pow(AC[1],2)), cvo.range, 1e-8);
  EXPECT_NEAR(CTanPoint.normalized()[0], AC.normalized()[0], 1e-8);
  EXPECT_NEAR(CTanPoint.normalized()[1], AC.normalized()[1], 1e-8);

}

TEST(PositionAndVelocityOfTwoVehiclesWithUncertainty, WhenCalculatingVOCone_ConeIsCorrectlyCalculated)
{
  Eigen::Vector2d av1Pos(-4, -0.25);
  Eigen::Vector2d av1Vel(-2, -1);
  Eigen::Vector2d av2Pos(3, 7);
  Eigen::Vector2d av2Vel(-2, -3);

  Eigen::Vector2d posUncertainty(0,0);

  Eigen::Vector2d avR = av2Pos - av1Pos;

  CollisionVO cvo = CollisionVO();
  std::vector<Eigen::Vector2d> output;
  cvo.get_collision_cone(av1Pos, av1Vel, av2Pos, av2Vel, posUncertainty, output);

  // ASSERT_TRUE();

  // Eigen::Vector2d finalA = av1Pos +

  Eigen::Vector2d A = output[0];
  Eigen::Vector2d B = output[1];
  Eigen::Vector2d C = output[2];

  EXPECT_NEAR(A[0], (av1Pos + (av1Vel + av2Vel) / 2)[0], 1e-8);
  EXPECT_NEAR(A[1], (av1Pos + (av1Vel + av2Vel) / 2)[1], 1e-8);

  Eigen::Vector2d AB = B - A;
  Eigen::Rotation2D<double> rotateB(PI/2);
  Eigen::Vector2d BTanPoint = avR + (rotateB*AB).normalized()*2*cvo.collisionRadius;

  EXPECT_NEAR(std::sqrt(std::pow(AB[0],2)+std::pow(AB[1],2)), cvo.range, 1e-8);
  EXPECT_NEAR(BTanPoint.normalized()[0], AB.normalized()[0], 1e-8);
  EXPECT_NEAR(BTanPoint.normalized()[1], AB.normalized()[1], 1e-8);

  Eigen::Vector2d AC = C - A;
  Eigen::Rotation2D<double> rotateC(-PI/2);
  Eigen::Vector2d CTanPoint = avR + (rotateC*AC).normalized()*2*cvo.collisionRadius;

  EXPECT_NEAR(std::sqrt(std::pow(AC[0],2)+std::pow(AC[1],2)), cvo.range, 1e-8);
  EXPECT_NEAR(CTanPoint.normalized()[0], AC.normalized()[0], 1e-8);
  EXPECT_NEAR(CTanPoint.normalized()[1], AC.normalized()[1], 1e-8);


  posUncertainty = Eigen::Vector2d(1,0.5);

  cvo.get_collision_cone(av1Pos, av1Vel, av2Pos, av2Vel, posUncertainty, output);

  // ASSERT_TRUE();

  // Eigen::Vector2d finalA = av1Pos +

  A = output[0];
  B = output[1];
  C = output[2];

  EXPECT_NEAR(A[0], (av1Pos + (av1Vel + av2Vel) / 2)[0], 1e-8);
  EXPECT_NEAR(A[1], (av1Pos + (av1Vel + av2Vel) / 2)[1], 1e-8);

  AB = B - A;
  rotateB = Eigen::Rotation2D<double>(PI/2);
  BTanPoint = avR + (rotateB*AB).normalized()*2*cvo.collisionRadius;

  EXPECT_NEAR(std::sqrt(std::pow(AB[0],2)+std::pow(AB[1],2)), cvo.range, 1e-8);
  // EXPECT_NEAR(BTanPoint.normalized()[0], AB.normalized()[0], 1e-8);
  // EXPECT_NEAR(BTanPoint.normalized()[1], AB.normalized()[1], 1e-8);

  AC = C - A;
  rotateC = Eigen::Rotation2D<double>(-PI/2);
  CTanPoint = avR + (rotateC*AC).normalized()*2*cvo.collisionRadius;

  EXPECT_NEAR(std::sqrt(std::pow(AC[0],2)+std::pow(AC[1],2)), cvo.range, 1e-8);
  // EXPECT_NEAR(CTanPoint.normalized()[0], AC.normalized()[0], 1e-8);
  // EXPECT_NEAR(CTanPoint.normalized()[1], AC.normalized()[1], 1e-8);

}


TEST(OnePtAndCollisionCone, WhenPointInsideCone_BarycentricFunctionReturnsTrue)
{
  Eigen::Vector2d av1Pos(0,0);
  Eigen::Vector2d av1Vel(1,1);
  Eigen::Vector2d av2Pos(6,3);
  Eigen::Vector2d av2Vel(-1,0);

  Eigen::Vector2d posUncertainty(0,0);

  CollisionVO cvo = CollisionVO();
  std::vector<Eigen::Vector2d> points;
  cvo.get_collision_cone(av1Pos, av1Vel, av2Pos, av2Vel, posUncertainty, points);

  EXPECT_TRUE(barycentric(points[0], points[1], points[2], av1Pos+av1Vel));
}

TEST(PositionAndVelocityOfOneOtherVehicle, WhenVelocityIsInsideTheirCone_AVelocityOtherThanExpectedIsChosen)
{
  Eigen::Vector2d av1Pos(0,0);
  Eigen::Vector2d av1Vel(1,1);
  Eigen::Vector2d av2Pos(4,2);
  Eigen::Vector2d av2Vel(-1,0);

  Eigen::Vector2d uncertainty(0,0);

  std::vector<Eigen::Vector2d> otherPos{av2Pos};
  std::vector<Eigen::Vector2d> otherVel{av2Vel};
  std::vector<Eigen::Vector2d> posUncertainty{uncertainty};
  std::vector<Eigen::Vector2d> velUncertainty{uncertainty};
  CollisionVO cvo = CollisionVO();

  double numPoints{4};
  double dt(0.1);
  Eigen::VectorXd state(13);
  state << 0,0,0, 1.0,1.0,0, 0,0,0,0,  0,0,0;
  std::vector<Eigen::Vector2d> admissibleVelocities;
  set_admissible_velocities(numPoints,state,dt,admissibleVelocities);

  cvo.admissibleVelocities = admissibleVelocities;

  Eigen::Vector2d newVel = cvo.get_best_vel(av1Pos, av1Vel, av1Vel, state, dt,
      otherPos, otherVel, posUncertainty, velUncertainty);

  EXPECT_FALSE(newVel[0] == 1.0);
  EXPECT_FALSE(newVel[1] == 1.0);
}

TEST(PositionAndVelocityOfThreeOtherVehicles, WhenVelocityIsOutsideTheirCone_VelocityStaysTheSame)
{
  Eigen::Vector2d av1Pos(0,0);
  Eigen::Vector2d av1Vel(1,1);
  Eigen::Vector2d av2Pos(7,6);
  Eigen::Vector2d av2Vel(1.0,1.0);
  Eigen::Vector2d av3Pos(10,0);
  Eigen::Vector2d av3Vel(1.0,1.0);
  Eigen::Vector2d av4Pos(-7,6);
  Eigen::Vector2d av4Vel(1.0,1.0);

  Eigen::Vector2d uncertainty(0,0);

  std::vector<Eigen::Vector2d> otherPos{av2Pos,av3Pos,av4Pos};
  std::vector<Eigen::Vector2d> otherVel{av2Vel,av3Vel,av4Vel};

  std::vector<Eigen::Vector2d> velUncertainty{uncertainty, uncertainty, uncertainty};
  std::vector<Eigen::Vector2d> posUncertainty{uncertainty, uncertainty, uncertainty};

  CollisionVO cvo = CollisionVO();

  double numPoints{4};
  double dt(0.1);
  Eigen::VectorXd state(13);
  state << 0,0,0, 1.0,1.0,0, 0,0,0,0,  0,0,0;
  std::vector<Eigen::Vector2d> admissibleVelocities;
  set_admissible_velocities(numPoints,state,dt,admissibleVelocities);

  cvo.admissibleVelocities = admissibleVelocities;

  Eigen::Vector2d newVel = cvo.get_best_vel(av1Pos, av1Vel, av1Vel, state, dt,
    otherPos, otherVel, posUncertainty, velUncertainty);

  EXPECT_EQ(newVel[0],1);
  EXPECT_EQ(newVel[1],1);
}

// TEST(AccelerationAndTimeStep, WhenCalulatingAdmissibleVelcities_TheValuesAreWithinExpectedRadius)
// {
//   CollisionVO cvo = CollisionVO();
//
//   for(int i(0);i<cvo.admissibleVelocities.size();i++)
//   {
//     EXPECT_TRUE(cvo.admissibleVelocities[i].norm() <= cvo.Ts * cvo.maxAccel);
//   }
//
//   cvo.Ts = 0.001;
//   cvo.maxAccel = 50;
//   cvo.set_admissible_velocities();
//
//   for(int i(0);i<cvo.admissibleVelocities.size();i++)
//   {
//     EXPECT_TRUE(cvo.admissibleVelocities[i].norm() <= cvo.Ts * cvo.maxAccel);
//   }
// }

TEST(PositionAndVelocityOfAnotherVehicle, WhenCalulatingCollisionTime_TheValuesArePositive)
{

    Eigen::Vector2d av1Pos(0,0);
    Eigen::Vector2d av1Vel(1,1);
    Eigen::Vector2d av2Pos(6,3);
    Eigen::Vector2d av2Vel(-1,0);

    CollisionVO cvo = CollisionVO();

    Eigen::Vector2d Pt{0.1,0.1};
    double collisionTime = cvo.get_collision_time(av1Pos, av1Vel, av2Pos, av2Vel, av1Pos+av1Vel+Pt);
    // std::cout << collisionTime << std::endl;
    EXPECT_TRUE(collisionTime > 0);

}

TEST(FourPoints, WhenCalulatingIntersection_TheIntersectionIsZero)
{

  Eigen::Vector2d p1(-1,-2);
  Eigen::Vector2d p2(1,2);
  Eigen::Vector2d q1(-2,3);
  Eigen::Vector2d q2(2,-3);

  CollisionVO cvo = CollisionVO();

  Eigen::Vector2d intersect;
  cvo.get_intersect(p1,p2,q1,q2,intersect);

  EXPECT_NEAR(intersect[0],0,1e-8);
  EXPECT_NEAR(intersect[1],0,1e-8);
}

TEST(FourPointsWhichDontIntersect, WhenCalulatingIntersection_ErrorIsThrown)
{

  Eigen::Vector2d p1(-1,-1);
  Eigen::Vector2d p2(1,1);
  Eigen::Vector2d q1(2,2);
  Eigen::Vector2d q2(3,3);

  CollisionVO cvo = CollisionVO();

  Eigen::Vector2d intersect;

  try
  {
    cvo.get_intersect(p1,p2,q1,q2,intersect);
  }
  catch(char const* err)
  {
    EXPECT_EQ(err,std::string("No Intersect!"));
  }

}

TEST(RVOTriangleAndUncertainty, WhenCalulatingVelocityUncertaintyTriangle_CorrectTriangleIsFormulated)
{

  Eigen::Vector2d A(0,0);
  Eigen::Vector2d B(0,10);
  Eigen::Vector2d C(10,0);
  Eigen::Vector2d velUncertainty(2,1);

  CollisionVO cvo = CollisionVO();

  std::vector<Eigen::Vector2d> uncertainTriangle;
  cvo.get_vel_uncertainty_tri(A,B,C,velUncertainty,uncertainTriangle);

  EXPECT_NEAR(uncertainTriangle[0][0],-2,1e-8);
  EXPECT_NEAR(uncertainTriangle[0][1],-1,1e-8);
}

TEST(RVOTriangleAndUncertainty, WhenCalulatingVelocityUncertaintyTriangle_PreviousAIsBarycentric)
{

  Eigen::Vector2d A(0,0);
  Eigen::Vector2d B(1,9);
  Eigen::Vector2d C(9,1);
  Eigen::Vector2d velUncertainty(2,1);

  CollisionVO cvo = CollisionVO();

  std::vector<Eigen::Vector2d> uncertainTriangle;
  cvo.get_vel_uncertainty_tri(A,B,C,velUncertainty,uncertainTriangle);
  EXPECT_TRUE(barycentric(uncertainTriangle[0],  uncertainTriangle[1], uncertainTriangle[2], A));
}

TEST(RVOTriangleAndZeroUncertainty, WhenCalulatingVelocityUncertaintyTriangle_PreviousValuesAreRepresented)
{

  Eigen::Vector2d A(0,0);
  Eigen::Vector2d B(1,9);
  Eigen::Vector2d C(9,1);
  Eigen::Vector2d velUncertainty(0,0);

  CollisionVO cvo = CollisionVO();

  std::vector<Eigen::Vector2d> uncertainTriangle;
  cvo.get_vel_uncertainty_tri(A,B,C,velUncertainty,uncertainTriangle);
  EXPECT_NEAR(uncertainTriangle[0][0], A[0],1e-8);
  EXPECT_NEAR(uncertainTriangle[0][1], A[1],1e-8);
  EXPECT_NEAR(uncertainTriangle[1][0], B[0],1e-8);
  EXPECT_NEAR(uncertainTriangle[1][1], B[1],1e-8);
  EXPECT_NEAR(uncertainTriangle[2][0], C[0],1e-8);
  EXPECT_NEAR(uncertainTriangle[2][1], C[1],1e-8);
}

TEST(PositionVelocityAndUncertaintyOfAnotherVehicles, WhenVelocityIsOutsideTheirConeButInsideUncertainty_VelocityChanges)
{
  Eigen::Vector2d av1Pos(0,0);
  Eigen::Vector2d av1Vel(1,1);
  Eigen::Vector2d av2Pos(10,10);
  Eigen::Vector2d av2Vel(0.5,0.0);

  Eigen::Vector2d noUncertainty(0,0);

  std::vector<Eigen::Vector2d> otherPos{av2Pos};
  std::vector<Eigen::Vector2d> otherVel{av2Vel};

  std::vector<Eigen::Vector2d> posUncertainty{noUncertainty};
  std::vector<Eigen::Vector2d> velUncertainty{noUncertainty};

  CollisionVO cvo = CollisionVO();

  double numPoints{4};
  double dt(0.1);
  Eigen::VectorXd state(13);
  state << 0,0,0, 1.0,1.0,0, 0,0,0,0,  0,0,0;
  std::vector<Eigen::Vector2d> admissibleVelocities;
  set_admissible_velocities(numPoints,state,dt,admissibleVelocities);

  cvo.admissibleVelocities = admissibleVelocities;

  Eigen::Vector2d newVel = cvo.get_best_vel(av1Pos, av1Vel, av1Vel, state, dt,
    otherPos, otherVel, posUncertainty, velUncertainty);

  EXPECT_EQ(newVel[0],1);
  EXPECT_EQ(newVel[1],1);

  Eigen::Vector2d yesUncertainty(10,10);
  std::vector<Eigen::Vector2d> velUncertainty2{yesUncertainty};

  newVel = cvo.get_best_vel(av1Pos, av1Vel, av1Vel, state, dt,
    otherPos, otherVel, posUncertainty, velUncertainty2);

  EXPECT_FALSE(newVel[0] == 1.0);
  EXPECT_FALSE(newVel[1] == 1.0);

  std::vector<Eigen::Vector2d> posUncertainty2{yesUncertainty};

  newVel = cvo.get_best_vel(av1Pos, av1Vel, av1Vel, state, dt,
    otherPos, otherVel, posUncertainty2, velUncertainty);

  EXPECT_FALSE(newVel[0] == 1.0);
  EXPECT_FALSE(newVel[1] == 1.0);
}

TEST(ThreeColinearPoints, WhenCalculatingOrientation_FunctionReturnsZero)
{
  Eigen::Vector2d A(-1.5,1);
  Eigen::Vector2d B(-0.5,0);
  Eigen::Vector2d C(1,-1.5);

  int orientation = get_orientation(A,B,C);

  EXPECT_EQ(orientation,0);
}


TEST(ThreeClockwisePoints, WhenCalculatingOrientation_FunctionReturnsOne)
{
  Eigen::Vector2d A(-5,-3);
  Eigen::Vector2d B(-1,1);
  Eigen::Vector2d C(1,-2);

  int orientation = get_orientation(A,B,C);

  EXPECT_EQ(orientation,1);
}

TEST(AValueAndEllipseDimensions, WhenCalculatingUncertaintyCone_TheValuesAreColinear)
{
  Eigen::Vector2d A(-2,-6);
  Eigen::Vector2d B;
  Eigen::Vector2d C;
  double a(10);
  double b(10);

  CollisionVO cvo = CollisionVO();

  cvo.get_pos_uncertainty_cone(A,B,C,a,b);

  int orientation = get_orientation(A,B,C);

  int colinear(0);

  EXPECT_EQ(orientation, colinear);
}
