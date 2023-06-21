#pragma once

#include "common_cpp/common.h"
#include "vehicle.h"
#include <fstream>


using namespace Eigen;


namespace quadrotor
{

class Controller
{

public:

  Controller();
  ~Controller();

  void load(const std::string &filename,
    std::vector<double> loaded_wps,
    const bool &use_random_seed,
    const std::string &name,
    double max_vel = 0.0);
  quadrotor::uVector computeControl(const vehicle::Stated &x,
    const double t,
    vehicle::Stated xc);
  inline vehicle::Stated getCommandedState() const { return xc_; }

  typedef struct
  {
    double roll;
    double pitch;
    double yaw_rate;
    double throttle;
    double vel;
  } max_t;

  struct PID
  {
    PID();
    void init(float kp, float ki, float kd, float max, float min, float tau);
    float run(float dt, float x, float x_c, bool update_integrator, bool update);
    float run(float dt, float x, float x_c, bool update_integrator, float xdot, bool update);

    float kp_;
    float ki_;
    float kd_;

    float max_;

    float integrator_;
    float differentiator_;
    float prev_x_;
    float tau_;
  };
  
  PID roll_;
  PID pitch_;
  PID yaw_rate_;
private:





  // Parameters
  double throttle_eq_;
  double mass_;
  double max_thrust_;
  int path_type_;
  std::default_random_engine rng_;

  // // Waypoint Parameters
  // MatrixXd waypoints_;
  // int current_waypoint_id_;
  // double waypoint_threshold_;
  // double waypoint_velocity_threshold_;
  //
  // // Trajectory Parameters
  // double traj_delta_north_;
  // double traj_delta_east_;
  // double traj_delta_alt_;
  // double traj_delta_yaw_;
  // double traj_nom_north_;
  // double traj_nom_east_;
  // double traj_nom_alt_;
  // double traj_nom_yaw_;
  // double traj_north_freq_;
  // double traj_east_freq_;
  // double traj_alt_freq_;
  // double traj_yaw_freq_;
  //
  // // Circumnavigation parameters
  // double circ_rd_;
  // double circ_hd_;
  // double circ_kr_;
  // double circ_kp_;
  // double circ_kh_;

  // Controller Gains
  Matrix3d K_p_; // position
  Matrix3d K_v_; // velocity
  Matrix3d K_d_; // disturbance acceleration


  // Memory for sharing information between functions
  vehicle::Stated xc_; // command
  max_t max_ = {};
  double prev_time_;
  // double vo_time_{0};
  // Matrix<double,3,1> xcVoVel_;
  uint8_t control_mode_;
  Vector3d dhat_; // disturbance acceleration

  // Target estimation parameters
  bool use_target_truth_, bearing_only_;
  Vector3d z_, vz_;
  double kz_, kvz_;
  Vector3d target_noise_;
  std::normal_distribution<double> target_noise_dist_;

  // Logging
  std::ofstream target_log_;
  std::ofstream command_state_log_;
  std::ofstream command_log_;
  std::ofstream euler_command_log_;

  // Functions
  // void updateWaypointManager();
  // void updateTrajectoryManager(const double &t);
  void log(const double& t, const uVector &u);
  // Matrix<double,3,1> calculate_VO(vehicle::Stated x,
  //   std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_positions,
  //   std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_velocities,
  //   Matrix<double,3,1> desVelocity);
};

} // namespace quadrotor
