#include "collision_vo/buffer.h"


//TODO: declare expected types
Eigen::Vector2d CalculateBuffer(const Eigen::Vector2d& host_pos,
				    const std::vector<Eigen::Vector2d>& invader_pos,
				    const double& collision_radius,
				    const double& power)
{
  //TODO: make sure this actually works, Emily
  int num_invaders = invader_pos.size();
  if(num_invaders==0)
      return Eigen::Vector2d{0,0};
  //FIiiiix this
  double buffer_radius = collision_radius;

  Eigen::Vector3d sum_velocity{0,0,0};

  for(int index = 0; index < num_invaders; index++)
  {
    //Get the difference in position of the host and current invader
    double x_pos = host_pos(0);
    double y_pos = host_pos(1);
    double z_pos = 0;

		Eigen::Vector3d host{x_pos,y_pos,z_pos};

    double x_inv = invader_pos[index](0);
    double y_inv = invader_pos[index](1);
    double z_inv = 0;

		Eigen::Vector3d invader{x_inv,y_inv,z_inv};

		Eigen::Vector3d diff = invader - host;
    // double x_diff = x_inv - x_pos, y_diff = y_inv - y_pos, z_diff = z_inv - z_pos;

    double euc_dist = diff.norm(); //pow((pow(x_diff, 2.0) + pow(y_diff, 2.0) + pow(z_diff, 2.0)), (1.0/2.0));
    double dist_to_buffer = euc_dist - buffer_radius;
		if(dist_to_buffer == 0)
		{
			dist_to_buffer = 1e-8;
			std::cout << "here\n";
		}
    //  buffer_force = abs((1/dist_to_buffer*(host.buffer_radius**(1/self.power)))**self.power)

    double buffer_force = pow(buffer_radius/dist_to_buffer, power); // pow(1.0/dist_to_buffer*pow(buffer_radius, (1.0/power)), power);

		if(isinf(buffer_force))
			std::cout << "here2\n";
    // double closest_buffer_point[] = {x_diff*dist_to_buffer/euc_dist,
    //                 y_diff*dist_to_buffer/euc_dist,
    //                 z_diff*dist_to_buffer/euc_dist};
		//
    // double buffer_direction[] = {x_diff >= 0 ? -1.0 : 1.0,
		// 		y_diff >= 0 ? -1.0 : 1.0,
		// 		z_diff >= 0 ? -1.0 : 1.0};

    Eigen::Vector3d buffer_velocity;

	  buffer_velocity = abs(buffer_force) * (host-invader).normalized(); //(buffer_force/dist_to_buffer)*buffer_direction[jindex]*closest_buffer_point[jindex];

		sum_velocity += buffer_velocity;
  }

  Eigen::Vector3d avg_velocity{0.0, 0.0, 0.0};

  // for(int lindex = 0; lindex < 2; lindex++)
  // {
  //   avg_velocity[lindex] = sum_velocity[lindex]/(1.0*num_invaders);
  // }

	avg_velocity = sum_velocity / (1.0*num_invaders);


  return avg_velocity.segment<2>(0);
}
