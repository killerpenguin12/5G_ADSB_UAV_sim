#include "common_cpp/common.h"
#include "common_cpp/progress_bar.h"
#include "quadrotor.h"
// #include "fixed_wing.h"
// #include "bicycle.h"
#include "environment.h"



int main()
{
  bool use_random_seed;
  double t(0), tf, dt;
  common::get_yaml_node("use_random_seed", "../params/sim.yaml", use_random_seed);
  common::get_yaml_node("tf", "../params/sim.yaml", tf);
  common::get_yaml_node("dt", "../params/sim.yaml", dt);
  if (use_random_seed)
    std::srand((unsigned)std::time(NULL));

  // Create progress bar
  common::ProgressBar prog_bar;
  prog_bar.init(tf/dt,40);

  // Create environment
  environment::Environment env("../params/sim.yaml");

  // Create vehicles -- last input is vehicle ID
  quadrotor::Quadrotor quad1("../params/quadrotor.yaml", "../params/waypoints1.yaml", env, use_random_seed, 0);
  quadrotor::Quadrotor quad2("../params/quadrotor.yaml", "../params/waypoints2.yaml", env, use_random_seed, 0);
  // fixedwing::FixedWing wing1("../params/fixed_wing1.yaml", env, use_random_seed, 1);
  // bicycle::Bicycle bike1("../params/bicycle1.yaml", env, use_random_seed, 2);

  // Store initial vehicle positions in environment
  env.initVehicle(quad1.getState().p, quad1.id_);
  env.initVehicle(quad2.getState().p, quad2.id_);
  // env.initVehicle(wing1.getState().p, wing1.id_);
  // env.initVehicle(bike1.getState().p, bike1.id_);

  // Main simulation loop
  while (t <= tf)
  {
    t += dt;
    prog_bar.print(t/dt);

    // Run each vehicle
    quad1.run(t, env);
    quad2.run(t, env);
//    wing1.run(t, env);
//    bike1.run(t, env);

    // Update wind and stored vehicle positions in environment
    env.updateWind(t);
    env.updateVehicle(quad1.getState().p, quad1.id_);
    env.updateVehicle(quad2.getState().p, quad2.id_);
    // env.updateVehicle(wing1.getState().p, wing1.id_);
    // env.updateVehicle(bike1.getState().p, bike1.id_);
  }
  // std::cout << quad1.getState().toEigen() << std::endl;

  prog_bar.finished();

  return 0;
}
