#include "environment.h"

namespace environment
{


Environment::Environment()
{
  t_prev_ = 0;
}


Environment::Environment(const std::string filename)
{
  t_prev_ = 0;
  load(filename);
}


Environment::~Environment()
{
  environment_log_.close();
  wind_log_.close();
}


void Environment::load(const std::string filename)
{
  // Initialize random number generator
  bool use_random_seed;
  common::get_yaml_node("use_random_seed", filename, use_random_seed);
  if (use_random_seed)
    rng_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());

  // Initialize wind and its walk parameters
  double vw_north_init_var, vw_east_init_var, vw_down_init_var;
  double vw_north_walk_stdev, vw_east_walk_stdev, vw_down_walk_stdev;
  common::get_yaml_node("enable_wind", filename, enable_wind_);
  common::get_yaml_node("random_initial_wind", filename, random_init_wind_);
  common::get_yaml_node("wind_north_init_stdev", filename, vw_north_init_var);
  common::get_yaml_node("wind_east_init_stdev", filename, vw_east_init_var);
  common::get_yaml_node("wind_down_init_stdev", filename, vw_down_init_var);
  common::get_yaml_node("wind_north_walk_stdev", filename, vw_north_walk_stdev);
  common::get_yaml_node("wind_east_walk_stdev", filename, vw_east_walk_stdev);
  common::get_yaml_node("wind_down_walk_stdev", filename, vw_down_walk_stdev);
  common::get_yaml_eigen("wind_init_vector", filename, vw_);
  vw_north_walk_dist_ = std::normal_distribution<double>(0.0,vw_north_walk_stdev);
  vw_east_walk_dist_ = std::normal_distribution<double>(0.0,vw_east_walk_stdev);
  vw_down_walk_dist_ = std::normal_distribution<double>(0.0,vw_down_walk_stdev);
  if (random_init_wind_)
  {
    vw_ = Eigen::Vector3d::Random();
    vw_(0) *= vw_north_init_var;
    vw_(1) *= vw_east_init_var;
    vw_(2) *= vw_down_init_var;
  }
  if (!enable_wind_)
    vw_.setZero();

  // Build the room or ground
  common::get_yaml_node("fly_indoors", filename, fly_indoors_);
  common::get_yaml_node("landmark_density", filename, lm_density_);
  common::get_yaml_node("landmark_deviation", filename, lm_deviation_);
  if (fly_indoors_)
  {
    common::get_yaml_node("indoor_north_dim", filename, indoor_north_dim_);
    common::get_yaml_node("indoor_east_dim", filename, indoor_east_dim_);
    common::get_yaml_node("indoor_height", filename, indoor_height_);
    buildRoom();
  }
  else
  {
    common::get_yaml_node("outdoor_north_dim", filename, outdoor_north_dim_);
    common::get_yaml_node("outdoor_east_dim", filename, outdoor_east_dim_);
    common::get_yaml_node("outdoor_height", filename, outdoor_height_);
    common::get_yaml_node("outdoor_hill_freq", filename, hill_freq_);
    buildGround();
  }

  // Initialize loggers
  environment_log_.open("/tmp/environment.log");
  wind_log_.open("/tmp/wind.log");

  // Log environment initial wind data
  environment_log_.write((char*)points_.data(), points_.rows() * points_.cols() * sizeof(double));
  logWind(0);
}


void Environment::buildRoom()
{
  // Origin is at the center of the room on the floor
  int num_pts_floor = indoor_north_dim_ * indoor_east_dim_ * lm_density_;
  int num_pts_ceil = indoor_north_dim_ * indoor_east_dim_ * lm_density_;
  int num_pts_north = indoor_height_ * indoor_east_dim_ * lm_density_;
  int num_pts_south = indoor_height_ * indoor_east_dim_ * lm_density_;
  int num_pts_east = indoor_height_ * indoor_north_dim_ * lm_density_;
  int num_pts_west = indoor_height_ * indoor_north_dim_ * lm_density_;
  int num_pts_total = num_pts_floor + num_pts_ceil + num_pts_north + num_pts_south + num_pts_east + num_pts_west;

  // Allocate space for all points
  points_.resize(3,num_pts_total);

  // Floor
  Eigen::ArrayXXd pts_floor(3,num_pts_floor);
  pts_floor.setRandom();
  pts_floor.row(0) *= indoor_north_dim_ / 2.0;
  pts_floor.row(1) *= indoor_east_dim_ / 2.0;
  pts_floor.row(2) *= lm_deviation_;

  // Ceiling
  Eigen::ArrayXXd pts_ceil(3,num_pts_ceil);
  pts_ceil.setRandom();
  pts_ceil.row(0) *= indoor_north_dim_ / 2.0;
  pts_ceil.row(1) *= indoor_east_dim_ / 2.0;
  pts_ceil.row(2) *= lm_deviation_;
  pts_ceil.row(2) -= indoor_height_; // Offset from origin

  // North wall
  Eigen::ArrayXXd pts_north(3,num_pts_north);
  pts_north.setRandom();
  pts_north.row(0) *= lm_deviation_;
  pts_north.row(1) *= indoor_east_dim_ / 2.0;
  pts_north.row(2) -= 1; // Shift random values to between 0 and -2
  pts_north.row(2) *= indoor_height_ / 2.0;
  pts_north.row(0) += indoor_north_dim_ / 2.0; // Offset from origin

  // South wall
  Eigen::ArrayXXd pts_south(3,num_pts_south);
  pts_south.setRandom();
  pts_south.row(0) *= lm_deviation_;
  pts_south.row(1) *= indoor_east_dim_ / 2.0;
  pts_south.row(2) -= 1; // Shift random values to between 0 and -2
  pts_south.row(2) *= indoor_height_ / 2.0;
  pts_south.row(0) -= indoor_north_dim_ / 2.0; // Offset from origin

  // East wall
  Eigen::ArrayXXd pts_east(3,num_pts_east);
  pts_east.setRandom();
  pts_east.row(0) *= indoor_north_dim_ / 2.0;
  pts_east.row(1) *= lm_deviation_;
  pts_east.row(2) -= 1; // Shift random values to between 0 and -2
  pts_east.row(2) *= indoor_height_ / 2.0;
  pts_east.row(1) += indoor_east_dim_ / 2.0; // Offset from origin

  // West wall
  Eigen::ArrayXXd pts_west(3,num_pts_west);
  pts_west.setRandom();
  pts_west.row(0) *= indoor_north_dim_ / 2.0;
  pts_west.row(1) *= lm_deviation_;
  pts_west.row(2) -= 1; // Shift random values to between 0 and -2
  pts_west.row(2) *= indoor_height_ / 2.0;
  pts_west.row(1) -= indoor_east_dim_ / 2.0; // Offset from origin

  // Concatenate all points
  points_ << pts_floor.matrix(), pts_ceil.matrix(), pts_north.matrix(),
             pts_south.matrix(), pts_east.matrix(), pts_west.matrix();
}


void Environment::buildGround()
{
  // Origin is at the center of the room on the floor
  int num_pts = outdoor_north_dim_ * outdoor_east_dim_ * lm_density_;

  // Allocate space for all points
  points_.resize(3,num_pts);

  // Create flat ground points
  Eigen::ArrayXXd pts_ground(3,num_pts);
  pts_ground.setRandom();
  pts_ground.row(0) *= outdoor_north_dim_ / 2.0;
  pts_ground.row(1) *= outdoor_east_dim_ / 2.0;
  pts_ground.row(2) *= lm_deviation_;

  // Add hills
  for (int i = 0; i < pts_ground.cols(); ++i)
    pts_ground.col(i).z() = getElevation(pts_ground.col(i).x(), pts_ground.col(i).y());

  // Concatenate all points
  points_ << pts_ground.matrix();
}


const double Environment::getElevation(const double &x, const double &y) const
{
  return outdoor_height_ * sin(hill_freq_ * x) + outdoor_height_ * sin(hill_freq_ * y);
}


void Environment::logWind(const double t)
{
  // Write data to binary files and plot in another program
  wind_log_.write((char*)&t, sizeof(double));
  wind_log_.write((char*)vw_.data(), vw_.rows() * sizeof(double));
}


void Environment::updateWind(const double t)
{
  if (enable_wind_)
  {
    double north_walk = vw_north_walk_dist_(rng_);
    double east_walk = vw_east_walk_dist_(rng_);
    double down_walk = vw_down_walk_dist_(rng_);
    vw_walk_ << north_walk, east_walk, down_walk;
    vw_ += vw_walk_ * (t - t_prev_);
  }
  t_prev_ = t;
  logWind(t);
}


void Environment::initVehicle(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const int& id)
{
  vehicle_positions_.push_back(p);
  vehicle_velocities_.push_back(v);
  if (vehicle_positions_.size() != id + 1)
    std::runtime_error("Vehicle ID does not match Environment conainer index!");
  ids_.push_back(id);
}


void Environment::updateVehicle(const Eigen::Vector3d &p, const Eigen::Vector3d &v, const int& id)
{
  vehicle_positions_[id] = p;
  vehicle_velocities_[id] = v;
}

const std::map<int, Eigen::VectorXd> Environment::get_all_vehicles() //maybe here would be a good place for blindness, cant see the vehicles
{
  std::map<int, Eigen::VectorXd> AllVehicles;
  for(int i(0);i<ids_.size();i++)
  {
      Eigen::VectorXd state(4);
      state(0) = vehicle_positions_[i][0];
      state(1) = vehicle_positions_[i][1];
      state(2) = vehicle_velocities_[i][0];
      state(3) = vehicle_velocities_[i][1];
      AllVehicles[ids_[i]] = state;
  }

  return AllVehicles;
}

}
