#include <gtest/gtest.h>
#include "collision_vo/convex_hull.h"


//next_to_top
TEST(AStackOfPoints, WhenObtainingNextToTop_CorrectPointIsReturned)
{
  Eigen::Vector2d A(-2,-6);
  Eigen::Vector2d B(3,4);
  Eigen::Vector2d C(5,-3);
  std::stack<Eigen::Vector2d> S;
  S.push(A);
  S.push(B);
  S.push(C);
  Eigen::Vector2d nextToTop = next_to_top(S);

  EXPECT_EQ(nextToTop[0], B[0]);
  EXPECT_EQ(nextToTop[1], B[1]);
}

//dist_square
TEST(TwoPoints, WhenCalculatingDistanceSquare_CorrectDistanceIsReturned)
{
  Eigen::Vector2d A(-2,-6);
  Eigen::Vector2d B(3,4);
  double distance = dist_square(A,B);

  EXPECT_EQ(distance, 125);
}

//convex_hull
TEST(ASetOfPoints, WhenConvexHull_CorrectPointsAreReturned)
{
  std::vector<Eigen::Vector2d> points = {Eigen::Vector2d{0, 3}, Eigen::Vector2d{1, 1}, Eigen::Vector2d{2, 2}, Eigen::Vector2d{4, 4},
					Eigen::Vector2d{0, 0}, Eigen::Vector2d{1, 2}, Eigen::Vector2d{3, 1}, Eigen::Vector2d{3, 3}};
	int n = points.size();
  std::vector<Eigen::Vector2d> returnPoints;
	convex_hull(points, n, returnPoints);
//   while (!S.empty())
//   {
//   	Eigen::Vector2d p = S.top();
//   	std::cout << "(" << p[0] << ", " << p[1] <<")" << std::endl;
//   	S.pop();
//   }

  EXPECT_EQ(returnPoints[0][0],0);
  EXPECT_EQ(returnPoints[0][1],3);

  EXPECT_EQ(returnPoints[1][0],4);
  EXPECT_EQ(returnPoints[1][1],4);

  EXPECT_EQ(returnPoints[2][0],3);
  EXPECT_EQ(returnPoints[2][1],1);

  EXPECT_EQ(returnPoints[3][0],0);
  EXPECT_EQ(returnPoints[3][1],0);
}

TEST(ASetOfPoints2, WhenConvexHull_CorrectPointsAreReturned)
{
  std::vector<Eigen::Vector2d> points = {
     Eigen::Vector2d{1.0, 1.0},
     Eigen::Vector2d{0.718224, 1.1},
     Eigen::Vector2d{0.443647, 1.2},
     Eigen::Vector2d{1.08178,  1.1},
     Eigen::Vector2d{0.8    ,  1.2},
     Eigen::Vector2d{0.518224, 1.3},
     Eigen::Vector2d{1.15635 , 1.2},
     Eigen::Vector2d{0.881776, 1.3},
     Eigen::Vector2d{0.6     , 1.4},
     Eigen::Vector2d{1.1     , 1.08178 },
     Eigen::Vector2d{0.818224, 1.17818 },
     Eigen::Vector2d{0.543647, 1.26752 },
     Eigen::Vector2d{1.18178 , 1.17818 },
     Eigen::Vector2d{0.9     , 1.28178 },
     Eigen::Vector2d{0.618224, 1.37818 },
     Eigen::Vector2d{1.25635 , 1.26752 },
     Eigen::Vector2d{0.981776, 1.37818 },
     Eigen::Vector2d{0.7     , 1.48178 },
     Eigen::Vector2d{1.2     , 1.15635 },
     Eigen::Vector2d{0.918224, 1.2493  },
     Eigen::Vector2d{0.643647, 1.32841 },
     Eigen::Vector2d{1.28178 , 1.2493  },
     Eigen::Vector2d{1.0      , 1.35635 },
     Eigen::Vector2d{0.718224, 1.4493  },
     Eigen::Vector2d{1.35635 , 1.32841 },
     Eigen::Vector2d{1.08178 , 1.4493  },
     Eigen::Vector2d{0.8     , 1.55635 },
     Eigen::Vector2d{1.1     , 0.718224},
     Eigen::Vector2d{0.818224, 0.821823},
     Eigen::Vector2d{0.543647, 0.932479},
     Eigen::Vector2d{1.18178 , 0.821823},
     Eigen::Vector2d{0.9     , 0.918224},
     Eigen::Vector2d{0.618224, 1.02182 },
     Eigen::Vector2d{1.25635 , 0.932479},
     Eigen::Vector2d{0.981776, 1.02182 },
     Eigen::Vector2d{0.7     , 1.11822 },
     Eigen::Vector2d{1.2     , 0.8     },
     Eigen::Vector2d{0.918224, 0.9     },
     Eigen::Vector2d{0.643647, 1.      },
     Eigen::Vector2d{1.28178 , 0.9     },
     Eigen::Vector2d{1.0      , 1.      },
     Eigen::Vector2d{0.718224, 1.1     },
     Eigen::Vector2d{1.35635 , 1.      },
     Eigen::Vector2d{1.08178 , 1.1     },
     Eigen::Vector2d{0.8     , 1.2     },
     Eigen::Vector2d{1.3     , 0.881776},
     Eigen::Vector2d{1.01822 , 0.978177},
     Eigen::Vector2d{0.743647, 1.06752 },
     Eigen::Vector2d{1.38178 , 0.978177},
     Eigen::Vector2d{1.1     , 1.08178 },
     Eigen::Vector2d{0.818224, 1.17818 },
     Eigen::Vector2d{1.45635 , 1.06752 },
     Eigen::Vector2d{1.18178 , 1.17818 },
     Eigen::Vector2d{0.9     , 1.28178 },
     Eigen::Vector2d{1.2     , 0.443647},
     Eigen::Vector2d{0.918224, 0.550703},
     Eigen::Vector2d{0.643647, 0.671591},
     Eigen::Vector2d{1.28178 , 0.550703},
     Eigen::Vector2d{1.      , 0.643647},
     Eigen::Vector2d{0.718224, 0.750703},
     Eigen::Vector2d{1.35635 , 0.671591},
     Eigen::Vector2d{1.08178 , 0.750703},
     Eigen::Vector2d{0.8     , 0.843647},
     Eigen::Vector2d{1.3     , 0.518224},
     Eigen::Vector2d{1.01822 , 0.621823},
     Eigen::Vector2d{0.743647, 0.732479},
     Eigen::Vector2d{1.38178 , 0.621823},
     Eigen::Vector2d{1.1     , 0.718224},
     Eigen::Vector2d{0.818224, 0.821823},
     Eigen::Vector2d{1.45635 , 0.732479},
     Eigen::Vector2d{1.18178 , 0.821823},
     Eigen::Vector2d{0.9     , 0.918224},
     Eigen::Vector2d{1.4     , 0.6     },
     Eigen::Vector2d{1.11822 , 0.7     },
     Eigen::Vector2d{0.843647, 0.8     },
     Eigen::Vector2d{1.48178 , 0.7     },
     Eigen::Vector2d{1.2     , 0.8     },
     Eigen::Vector2d{0.918224, 0.9     },
     Eigen::Vector2d{1.55635 , 0.8     },
     Eigen::Vector2d{1.28178 , 0.9     },
     Eigen::Vector2d{1.0      , 1.0    }};

	int n = points.size();
  std::vector<Eigen::Vector2d> convexHull;
	convex_hull(points, n, convexHull);

  std::vector<Eigen::Vector2d> truth = {
     Eigen::Vector2d{0.543647, 0.932479},
     Eigen::Vector2d{0.643647, 0.671591},
     Eigen::Vector2d{0.918224, 0.550703},
     Eigen::Vector2d{1.2     , 0.443647},
     Eigen::Vector2d{1.3     , 0.518224},
     Eigen::Vector2d{1.4     , 0.6     },
     Eigen::Vector2d{1.48178 , 0.7     },
     Eigen::Vector2d{1.55635 , 0.8     },
     Eigen::Vector2d{1.45635 , 1.06752 },
     Eigen::Vector2d{1.35635 , 1.32841 },
     Eigen::Vector2d{1.08178 , 1.4493  },
     Eigen::Vector2d{0.8     , 1.55635 },
     Eigen::Vector2d{0.7     , 1.48178 },
     Eigen::Vector2d{0.6     , 1.4     },
     Eigen::Vector2d{0.518224, 1.3     },
     Eigen::Vector2d{0.443647, 1.2     }};

  std::cout << "Size: " << convexHull.size() << std::endl;
  for(int i=0; i<convexHull.size(); i++)
  {
    EXPECT_EQ(convexHull[i][0], truth[convexHull.size()-1-i][0]);
    EXPECT_EQ(convexHull[i][1], truth[convexHull.size()-1-i][1]);
  }

}
