#pragma once

#include "common_cpp/common.h"
#include "vehicle.h"


using namespace Eigen;


namespace quadrotor
{


class WPManager
{

public:
  WPManager();
  ~WPManager();

  void load(const std::string& filename,
    std::vector<double> loaded_wps,
    const bool& use_random_seed,
    const std::string& name,
    const double& max_velocity=0);

  vehicle::Stated updateWaypointManager(vehicle::Stated x);
  void updateTrajectoryManager(vehicle::Stated xc, const double& t);

private:

  // Waypoint Enumerations
  enum
  {
    PX,
    PY,
    PZ,
    PSI
  };

  // Memory for sharing information between functions
  vehicle::Stated xhat_; // estimate
  bool initialized_;

  // Waypoint Parameters
  MatrixXd waypoints_;
  int current_waypoint_id_;
  double waypoint_threshold_;
  double waypoint_velocity_threshold_;

  // Trajectory Parameters
  double traj_delta_north_;
  double traj_delta_east_;
  double traj_delta_alt_;
  double traj_delta_yaw_;
  double traj_nom_north_;
  double traj_nom_east_;
  double traj_nom_alt_;
  double traj_nom_yaw_;
  double traj_north_freq_;
  double traj_east_freq_;
  double traj_alt_freq_;
  double traj_yaw_freq_;

  // Circumnavigation parameters
  double circ_rd_;
  double circ_hd_;
  double circ_kr_;
  double circ_kp_;
  double circ_kh_;

  Matrix3d K_p_; // velocity

  double max_vel_;

};

}
