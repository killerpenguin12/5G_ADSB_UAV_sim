#pragma once

#include <fstream>
#include <random>
#include <chrono>
#include <eigen3/Eigen/Eigen>

#include "common_cpp/common.h"


namespace environment
{


class Environment
{

public:

  Environment();
  Environment(const std::string filename);
  ~Environment();

  void load(const std::string filename);
  void updateWind(const double t);
  void initVehicle(const Eigen::Vector3d& p, const Eigen::Vector3d &v, const int &id);
  void updateVehicle(const Eigen::Vector3d& p, const Eigen::Vector3d &v, const int &id);
  void logWind(const double t);
  const Eigen::MatrixXd& get_points() const { return points_; }
  const Eigen::Vector3d& get_vw() const { return vw_; }
  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& getVehiclePositions() const { return vehicle_positions_; }
  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& getVehicleVelocities() const { return vehicle_velocities_; }
  const std::vector<double> getIds() const { return ids_; }
  const double getElevation(const double& x, const double& y) const;

  const std::map<int, Eigen::VectorXd> get_all_vehicles();
  // const std::map<double, Eigen::VectorXd,
  //       std::less<int>,
  //       Eigen::aligned_allocator<std::pair<const double, Eigen::VectorXd>> > get_all_vehicles();


private:

  void buildRoom();
  void buildGround();

  std::default_random_engine rng_;
  Eigen::MatrixXd points_;
  bool fly_indoors_;
  double t_prev_;
  double lm_density_;
  double lm_deviation_;
  double indoor_north_dim_, indoor_east_dim_, indoor_height_;
  double outdoor_north_dim_, outdoor_east_dim_, outdoor_height_, hill_freq_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vehicle_positions_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vehicle_velocities_;
  std::vector<double> ids_;

  bool enable_wind_, random_init_wind_;
  Eigen::Vector3d vw_, vw_walk_;
  std::normal_distribution<double> vw_north_walk_dist_;
  std::normal_distribution<double> vw_east_walk_dist_;
  std::normal_distribution<double> vw_down_walk_dist_;

  std::ofstream environment_log_;
  std::ofstream wind_log_;

};


}
