#ifndef BUFFER_H
#define BUFFER_H

#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <iostream>

Eigen::Vector2d CalculateBuffer(const Eigen::Vector2d& host_pos,
				    const std::vector<Eigen::Vector2d>& invader_pos,
				    const double& collision_radius,
				    const double& power);

#endif // BUFFER_H
