#include "collision_vo/convex_hull.h"

// A global point needed for sorting points with reference
// to the first point Used in compare function of qsort()
Eigen::Vector2d p0;

Eigen::Vector2d next_to_top(std::stack<Eigen::Vector2d> &S)
{
  // A utility function to find next to top in a stack
	Eigen::Vector2d p = S.top();
	S.pop();
	Eigen::Vector2d res = S.top();
	S.push(p);
	return res;
}

int dist_square(const Eigen::Vector2d& p1,
  const Eigen::Vector2d& p2)
{
  // A utility function to return square of distance
  // between p1 and p2
	return (p1[0] - p2[0])*(p1[0] - p2[0]) +
		(p1[1] - p2[1])*(p1[1] - p2[1]);
}



int compare(const void *vp1, const void *vp2)
{
  // A function used by library function qsort() to sort an array of
  // points with respect to the first point
  Eigen::Vector2d *p1 = (Eigen::Vector2d *)vp1;
  Eigen::Vector2d *p2 = (Eigen::Vector2d *)vp2;

  // Find orientation
  int o = get_orientation(p0, *p1, *p2);
  if (o == 0)
  	return (dist_square(p0, *p2) >= dist_square(p0, *p1))? -1 : 1;

  return (o == 2)? -1: 1;
}

void convex_hull(std::vector<Eigen::Vector2d> points,
	int n,
	std::vector<Eigen::Vector2d>& returnVec)
{
	std::stack<Eigen::Vector2d> S;
  // Finds convex hull of a set of n points.

  // Find the bottommost point
  int ymin = points[0][1], min = 0;
  for (int i = 1; i < n; i++)
  {
  	int y = points[i][1];

  	// Pick the bottom-most or chose the left
  	// most point in case of tie
  	if ((y < ymin) || (ymin == y &&
  		points[i][0] < points[min][0]))
  		ymin = points[i][1], min = i;
  }

  // Place the bottom-most point at first position
  switch_points(points[0], points[min]);

  // Sort n-1 points with respect to the first point.
  // A point p1 comes before p2 in sorted output if p2
  // has larger polar angle (in counterclockwise
  // direction) than p1
  p0 = points[0];
  std::qsort(&points[1], n-1, sizeof(Eigen::Vector2d), compare);

  // If two or more points make same angle with p0,
  // Remove all but the one that is farthest from p0
  // Remember that, in above sorting, our criteria was
  // to keep the farthest point at the end when more than
  // one points have same angle.
  int m = 1; // Initialize size of modified array
  for (int i=1; i<n; i++)
  {
  	// Keep removing i while angle of i and i+1 is same
  	// with respect to p0
  	while (i < n-1 && get_orientation(p0, points[i], points[i+1]) == 0)
		{
			i++;
		}


  	points[m] = points[i];
  	m++; // Update size of modified array
  }

  // If modified array of points has less than 3 points,
  // convex hull is not possible
  if (m < 3) return;

  // Create an empty stack and push first three points
  // to it.
  // std::stack<Eigen::Vector2d> S;
  S.push(points[0]);
  S.push(points[1]);
  S.push(points[2]);

  // Process remaining n-3 points
  for (int i = 3; i < m; i++)
  {
  	// Keep removing top while the angle formed by
  	// points next-to-top, top, and points[i] makes
  	// a non-left turn
  	while (get_orientation(next_to_top(S), S.top(), points[i]) != 2)
  		S.pop();
  	S.push(points[i]);
  }

	while (!S.empty())
	{
			returnVec.push_back(S.top());
			S.pop();
	}

}
