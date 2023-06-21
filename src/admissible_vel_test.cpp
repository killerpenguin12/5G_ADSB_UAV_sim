#include "common_cpp/common.h"
#include "common_cpp/progress_bar.h"
#include "quadrotor.h"
#include "quad_control.h"
#include "quad_wp_manager.h"
#include "environment.h"

#include <vector>
#include <iostream>
#include "vehicle.h"

#include "collision_vo/admissible_velocities.h"
int main(int argc, char** argv)
{

  environment::Environment env("../params/sim.yaml");
  std::vector<double> waypoints;
  waypoints.push_back(-10.0);
  waypoints.push_back(0.0);
  waypoints.push_back(0.0);
  waypoints.push_back(0.0);
  waypoints.push_back(10.0);
  waypoints.push_back(0.0);
  waypoints.push_back(0.0);
  waypoints.push_back(0.0);
  std::string name = "quad_test";
  bool use_random_seed = false;

  int i = 0;

  quadrotor::Quadrotor* quad = new quadrotor::Quadrotor("../params/quadrotor.yaml",
      name,
      waypoints,
      env,
      use_random_seed,
      i);

  quadrotor::Controller* quadController = new quadrotor::Controller();
  quadController->load("../params/quadrotor.yaml",
      waypoints,
      use_random_seed,
      name);

  quadrotor::WPManager* quadWPManager = new quadrotor::WPManager();
  quadWPManager->load("../params/quadrotor.yaml",
      waypoints,
      use_random_seed,
      name);


  std::vector<Eigen::Vector2d> admissibleVelocities;

  double numPoints{3};

  Eigen::VectorXd state(13);
  state << -10,0,0, 1,0,0, 1,0,0,0, 0,0,0;

  double dt(0.1);
  set_admissible_velocities(numPoints,state,dt,admissibleVelocities);

  // std::cout << quad->getState().toEigen() << std::endl;

  // for(auto vel : admissibleVelocities)
  //   std::cout << vel << "\n\n";

  double dtcon(0.001);
  double t(0.0);
  double tf(0.2);

  // std::cout << "Init State: " << quad->getState().p << std::endl;
  std::cout << "Init State: " << quad->getState().v << std::endl;
  // std::cout << quad->getState().q << std::endl;
  // std::cout << quad->getState().omega << std::endl;

  // std::cout << "Ad: " << admissibleVelocities[20] << std::endl;
  //
  vehicle::Stated xc = quadWPManager->updateWaypointManager(quad->getState());
  xc.v[0] = admissibleVelocities[20][0];
  xc.v[1] = 0.1; //admissibleVelocities[20][1];
  std::cout << "XC: " << xc.v << std::endl;
  while (t <= tf)
  {
    t += dtcon;
    quadrotor::uVector u = quadController->computeControl(quad->getState(), t, xc);
    std::cout << "u: " << u << std::endl;
    // quadrotor::uVector u;
    // u(quadrotor::THRUST) = 1;
    // u(quadrotor::TAUX) = 200;
    // u(quadrotor::TAUY) = 50;
    // u(quadrotor::TAUZ) = 200;
    quad->run(t, env, u);
    std::cout << "State Vel: " << quad->getState().v[0] << " , " << quad->getState().v[1] << " T: " << t<< std::endl;
  }
  // std::cout << "State: " << quad->getState().p << std::endl;
  // std::cout << quad->getState().v << std::endl;
  // std::cout << quad->getState().q << std::endl;
  // std::cout << quad->getState().omega << std::endl;

  // std::cout << "State Quad: " << quad->getState().q << std::endl;
  // std::cout << "State Quad: " << quad->getState().q.euler() << std::endl;
}
