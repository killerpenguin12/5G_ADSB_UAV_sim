#ifndef COLLISIONVO_H
#define COLLISIONVO_H

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <iostream>
#include <omp.h>
#include <math.h>
#include <limits>
#include <stack>

#ifdef BUILD_COLLISION_TESTS
#include <gtest/gtest_prod.h>
#endif

#include "collision_vo/utils.h"
#include "collision_vo/admissible_velocities.h"
#include "collision_vo/convex_hull.h"
#include "collision_vo/min_solve.h"
#include "collision_vo/buffer.h"

typedef Eigen::Matrix<int,2,1> Vec2i;
typedef Eigen::Matrix<double,2,1> Vec2d;
typedef Eigen::Matrix<int,3,1> Vec3i;
typedef Eigen::Matrix<double,3,1> Vec3d;
typedef Eigen::Matrix<int,4,1> Vec4i;
typedef Eigen::Matrix<double,4,1> Vec4d;



enum Intersection { INTERSECT, NEGATIVE, NO_INTERSECT}; //, INSIDE };

class CollisionVO
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CollisionVO();

  ~CollisionVO();

  double collisionRadius{2.0};
  double range{10.0};
  double timeCollisionPenality{1.0};
  double maxAccel{100};
  double Ts{0.01};
  double uncertainPosPenality{1.0};
  double uncertainVelPenality{1.0};
  bool uncertaintyOn{false};
  // int numUncertaintyTri{10};
  std::vector<Vec2d> admissibleVelocities;
  double fmax{100.0};
  double numPointsAdmissible{3};
  double order{3};

  bool bufferOn{false};
  int bufferPower{20};

  Vec2d get_best_vel(const Vec2d& av1Pos,
    const Vec2d& av1Vel,
    const Vec2d& av1VelDes,
    const Eigen::VectorXd& state,
    const double& dt,
    const std::vector<Vec2d>& inRangePos,
    const std::vector<Vec2d>& inRangeVel,
    const std::vector<Vec2d>& posUncertainty,
    const std::vector<Vec2d>& velUncertainty);

private:

#ifdef BUILD_COLLISION_TESTS
  // Collision VO
  FRIEND_TEST(PositionAndVelocityOfTwoVehicles, WhenCalculatingVOCone_ConeIsCorrectlyCalculated);
  FRIEND_TEST(PositionAndVelocityOfTwoVehicles2, WhenCalculatingVOCone_ConeIsCorrectlyCalculated);
  FRIEND_TEST(PositionAndVelocityOfTwoVehiclesWithUncertainty, WhenCalculatingVOCone_ConeIsCorrectlyCalculated);
  FRIEND_TEST(OnePtAndCollisionCone, WhenPointInsideCone_BarycentricFunctionReturnsTrue);
  FRIEND_TEST(PositionAndVelocityOfOneOtherVehicle, WhenVelocityIsInsideTheirCone_AVelocityOtherThanExpectedIsChosen);
  FRIEND_TEST(PositionAndVelocityOfThreeOtherVehicles, WhenVelocityIsOutsideTheirCone_VelocityStaysTheSame);
  FRIEND_TEST(PositionAndVelocityOfAnotherVehicle, WhenCalulatingCollisionTime_TheValuesArePositive);
  FRIEND_TEST(FourPoints, WhenCalulatingIntersection_TheIntersectionIsZero);
  FRIEND_TEST(FourPointsWhichDontIntersect, WhenCalulatingIntersection_ErrorIsThrown);
  FRIEND_TEST(RVOTriangleAndUncertainty, WhenCalulatingVelocityUncertaintyTriangle_CorrectTriangleIsFormulated);
  FRIEND_TEST(RVOTriangleAndUncertainty, WhenCalulatingVelocityUncertaintyTriangle_PreviousAIsBarycentric);
  FRIEND_TEST(RVOTriangleAndZeroUncertainty, WhenCalulatingVelocityUncertaintyTriangle_PreviousValuesAreRepresented);
  FRIEND_TEST(PositionVelocityAndUncertaintyOfAnotherVehicles, WhenVelocityIsOutsideTheirConeButInsideUncertainty_VelocityChanges);
  FRIEND_TEST(ThreeColinearPoints, WhenCalculatingOrientation_FunctionReturnsZero);
  FRIEND_TEST(ThreeClockwisePoints, WhenCalculatingOrientation_FunctionReturnsOne);
  FRIEND_TEST(AValueAndEllipseDimensions, WhenCalculatingUncertaintyCone_TheValuesAreColinear);
#endif


  Intersection get_distance(const Vec2d& av1Xo,
    const Vec2d& av1Vo,
    const Vec2d& av2Xo,
    const Vec2d& av2Vo,
    const Vec2d& point,
    double& distance);

  Intersection line_ellipse_intersect(const Vec2d& A,
    const Vec2d& P,
    const Vec2d& uncertainty,
    double& distance,
    double& omega);


  double get_collision_time(const double& distance,
    const Vec2d& av1Xo,
    const Vec2d& av1Vo,
    const Vec2d& av2Vo,
    const Vec2d& point);


  void get_admissible_velocities(const Eigen::VectorXd& state, int N=4);

};
#endif
