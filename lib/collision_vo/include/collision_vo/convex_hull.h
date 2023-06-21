#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <eigen3/Eigen/Eigen>
#include <stack>
#include <stdlib.h>
#include <iostream>
#include "collision_vo/utils.h"

Eigen::Vector2d next_to_top(std::stack<Eigen::Vector2d> &S);

int dist_square(const Eigen::Vector2d& p1,
  const Eigen::Vector2d& p2);

int compare(const void *vp1, const void *vp2);

void convex_hull(std::vector<Eigen::Vector2d> points,
	int n,
	std::vector<Eigen::Vector2d>& returnPoints);

#endif
