#include <gtest/gtest.h>
#include <vector>
#include "collision_vo/buffer.h"

TEST(PositionAndVelocityOfTwoVehiclesFarAway, WhenCalculatingBuffer_CorrectValuesAreCalculated)
{
  Eigen::Vector2d av1Pos(34,43);
  Eigen::Vector2d av2Pos(43,43);

  std::vector<Eigen::Vector2d> otherPos{av2Pos};

  Eigen::Vector2d vec = CalculateBuffer(av1Pos,
    otherPos,
    5,
    10);

    std::cout << vec << std::endl;


  // EXPECT_NEAR(-1.1431801306477128e-06, vec[0], 1e-8);
  // EXPECT_NEAR(-1.1431801306477128e-06, vec[1], 1e-8);

  // vec = CalculateBuffer(av1Pos,
  //   otherPos,
  //   2,
  //   3);
  //
  // std::cout << vec << std::endl;

  // av1Pos = Eigen::Vector2d(40.0375,40.0375);
  // Eigen::Vector2d av2Pos(43,43);
  // std::vector<Eigen::Vector2d> otherPos{av2Pos};

  // vec = CalculateBuffer(av1Pos,
  //   otherPos,
  //   2,
  //   4);


  // EXPECT_NEAR(-655691.0005545176, vec[0], 1e-8);
  // EXPECT_NEAR(-655691.0005545176, vec[1], 1e-8);

  // av1Pos = Eigen::Vector2d(39 ,43);
  //
  // vec = CalculateBuffer(av1Pos,
  //   otherPos,
  //   2,
  //   3);
  //
  // std::cout << vec << std::endl;

}
