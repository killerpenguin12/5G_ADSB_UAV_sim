#include "quad_wp_manager.h"


namespace quadrotor
{

WPManager::WPManager() :
  initialized_(false)
{

}

WPManager::~WPManager()
{

}

void WPManager::load(const std::string& filename,
  std::vector<double> loaded_wps,
  const bool& use_random_,
  const std::string& name,
  const double& max_velocity)
{
  int num_waypoints = std::floor(loaded_wps.size()/4.0);
  waypoints_ = Map<MatrixXd>(loaded_wps.data(), 4, num_waypoints);
  current_waypoint_id_ = 0;
  // std::cout << "Waypoints Loaded: " << waypoints_ << std::endl;

  common::get_yaml_node("waypoint_threshold", filename, waypoint_threshold_); //dont need
  common::get_yaml_node("waypoint_velocity_threshold", filename, waypoint_velocity_threshold_); //dont need

  double traj_north_period = 10, traj_east_period = 10, traj_alt_period = 20, traj_yaw_period = 20;
  // waypoint_threshold: 0.1 dont need
  // waypoint_velocity_threshold: 0.5 dont need

  // traj_delta_north: 10
  // traj_delta_east: 10
  // traj_delta_alt: 2
  // traj_delta_yaw: 0
  // traj_nom_north: 0
  // traj_nom_east: 0
  // traj_nom_alt: 5
  // traj_nom_yaw: 0
  // traj_north_period: 20
  // traj_east_period: 10
  // traj_alt_period: 20
  // traj_yaw_period: 20
  // common::get_yaml_node("traj_delta_north", filename, traj_delta_north_);
  // common::get_yaml_node("traj_delta_east", filename, traj_delta_east_);
  // common::get_yaml_node("traj_delta_alt", filename, traj_delta_alt_);
  // common::get_yaml_node("traj_delta_yaw", filename, traj_delta_yaw_);
  // common::get_yaml_node("traj_nom_north", filename, traj_nom_north_);
  // common::get_yaml_node("traj_nom_east", filename, traj_nom_east_);
  // common::get_yaml_node("traj_nom_alt", filename, traj_nom_alt_);
  // common::get_yaml_node("traj_nom_yaw", filename, traj_nom_yaw_);
  // common::get_yaml_node("traj_north_period", filename, traj_north_period);
  // common::get_yaml_node("traj_east_period", filename, traj_east_period);
  // common::get_yaml_node("traj_alt_period", filename, traj_alt_period);
  // common::get_yaml_node("traj_yaw_period", filename, traj_yaw_period);
  traj_delta_north_ = 10;
  traj_delta_east_ = 10;
  traj_delta_alt_ = 2;
  traj_delta_yaw_ = 0;
  traj_nom_north_ = 0;
  traj_nom_east_ = 0;
  traj_nom_alt_ = 5;
  traj_nom_yaw_ = 0;
  traj_north_period = 20;
  traj_east_period = 10;
  traj_alt_period = 20;
  traj_yaw_period = 20;
  traj_north_freq_ = 2.0 * M_PI / traj_north_period;
  traj_east_freq_ = 2.0 * M_PI / traj_east_period;
  traj_alt_freq_ = 2.0 * M_PI / traj_alt_period;
  traj_yaw_freq_ = 2.0 * M_PI / traj_yaw_period;
  circ_rd_ = 5.0;
  circ_hd_ = 5.0;
  circ_kr_ = 5.0;
  circ_kp_ = 5.0;
  circ_kh_ = 5.0;
  // common::get_yaml_node("circ_rd", filename, circ_rd_);
  // common::get_yaml_node("circ_hd", filename, circ_hd_);
  // common::get_yaml_node("circ_kr", filename, circ_kr_);
  // common::get_yaml_node("circ_kp", filename, circ_kp_);
  // common::get_yaml_node("circ_kh", filename, circ_kh_);
  Vector3d Kp_diag(0.15,0.15,0.15);
  //K_p_ = [0.15, 0.15, 0.15]
  //if (Kp_diag)//common::get_yaml_eigen("Kp", filename, Kp_diag))
  K_p_ = Kp_diag.asDiagonal();

  if(max_velocity == 0.0){
    common::get_yaml_node("max_vel", filename, max_vel_);
    //max_vel_ =  5; Coulton Change this in super
    }
  else
    max_vel_ = max_velocity;
  //std::cout << "Max_vel: " << max_vel_ << std::endl;
}

vehicle::Stated WPManager::updateWaypointManager(vehicle::Stated x)
{
  // Update state estimate for waypoint manager
  xhat_ = x;

  vehicle::Stated xc;

  if (!initialized_)
  {
    initialized_ = true;
    Map<Vector4d> new_waypoint(waypoints_.block<4,1>(0, 0).data());
    xc.p = new_waypoint.segment<3>(PX);
    xc.q = quat::Quatd(0, 0, new_waypoint(PSI));
  }

  // Find the distance to the desired waypoint
  Vector4d current_waypoint = waypoints_.block<4,1>(0, current_waypoint_id_+1);
  Vector4d error;
  error.segment<3>(PX) = current_waypoint.segment<3>(PX) - xhat_.p;
  // error(PSI) = common::wrapAngle(current_waypoint(PSI) - xhat_.q.yaw(), 2.0 * M_PI);
  error(PSI) = 0;
  //Coulton this is the one that will push the UAVs to a new point in the random groups
  if (error.norm() < waypoint_threshold_  && xhat_.v.norm() < waypoint_velocity_threshold_)
   {
     // increment waypoint
     //current_waypoint_id_ = (current_waypoint_id_ + 1) % waypoints_.cols();
      waypoints_.block<4,1>(0, current_waypoint_id_+1) = -waypoints_.block<4,1>(0, current_waypoint_id_+1);
      waypoints_.block<4,1>(0, current_waypoint_id_) = -waypoints_.block<4,1>(0, current_waypoint_id_);
      Map<Vector4d> new_waypoint(waypoints_.block<4,1>(0, current_waypoint_id_+1).data());
   }
   //Coulton this is the end
  // Update The commanded State
  Map<Vector4d> new_waypoint(waypoints_.block<4,1>(0, current_waypoint_id_+1).data());
  xc.p = new_waypoint.segment<3>(PX);

  double psi = x.q.yaw();
  Matrix3d R_v_to_v1 = common::R_v_to_v1(psi);
  xc.v = R_v_to_v1 * K_p_ * (xc.p - x.p);
  //std::cout << "vel xc: " << xc.v << std::endl; 
  //std::cout << "pos xc: " << xc.p << std::endl; 
  //std::cout << "vel x: " << xc.v.norm() << std::endl; 
  //std::cout << "pos x: " << x.p << std::endl;
  if(xc.v.norm() > max_vel_)
    xc.v = xc.v.normalized()*max_vel_;
  //std::cout << "AFTER " << std::endl;
  /*std::cout << "Quad WP Manager"<< std::endl;
  std::cout << "vel xc: " << xc.v << std::endl; 
  std::cout << "pos xc: " << xc.p << std::endl; 
  std::cout << "vel x: " << x.v << std::endl; 
  std::cout << "pos x: " << x.p << std::endl;
  std::cout << "############################" << std::endl;*/
  xc.q = quat::Quatd(0, 0, new_waypoint(PSI));
  // xc.q = quat::Quatd(0,0,std::atan2((xc.p - x.p)[1], (xc.p - x.p)[0]));

  return xc;
}


void WPManager::updateTrajectoryManager(vehicle::Stated xc, const double& t)
{
  xc.p = Vector3d(traj_nom_north_ + traj_delta_north_ / 2.0 * cos(traj_north_freq_ * t),
                   traj_nom_east_ + traj_delta_east_ / 2.0 * sin(traj_east_freq_ * t),
                   -(traj_nom_alt_ + traj_delta_alt_ / 2.0 * sin(traj_alt_freq_ * t)));
  xc.q = quat::Quatd(0, 0, traj_nom_yaw_ + traj_delta_yaw_ / 2.0 * sin(traj_yaw_freq_ * t));
}

} // namespace quadrotor
