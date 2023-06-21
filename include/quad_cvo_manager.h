#pragma once

#include "common_cpp/common.h"
#include "collision_vo/collision_vo.h"
#include "gen_kalman_filter.h"
#include "vehicle.h"
#include "environment.h"
//#include "uatModel.h"


using namespace Eigen;


namespace quadrotor
{


class CVOManager
{

public:
  CVOManager();
  ~CVOManager();

  void load(const std::string &filename,
    environment::Environment* env,
    double id,
    double kalman_dt,
    double dt,
    double randRange,
    double maxAccel);

  void get_best_vel(const vehicle::Stated& x,
    const double t,
    vehicle::Stated& xc,
    std::vector<int> visibilityList);
  
  // void get_best_vel(const vehicle::Stated& x,
  // const double t,
  // vehicle::Stated& xc,
  // uat::UATModel *model,
  // std::vector<int> visibilityList,
  // std::vector<int> collisionList);

  void propagate(const double &t);

  std::map<int, GenKalmanFilter*> kalFilters;
  //std::vector<GenKalmanFilter*> kalFilters;
  // std::vector<double> ids;


private:

  void get_radar_uncertainty(const Eigen::Vector2d& p,
    Eigen::Vector2d& p_hat);

  CollisionVO quadCVO_;
  uint32_t id_;
  double kalman_dt_;
  double dt_;
  environment::Environment* env_;

  std::default_random_engine rng_;
  std::normal_distribution<double> position_dist_;
  std::normal_distribution<double> velocity_dist_;
  std::normal_distribution<double> range_dist_;
  std::normal_distribution<double> zenith_dist_;

  double drop_prob_;

  double sigmaQ_vel_ = 3;
  double alphaQ_vel_ = 0.5;
  double sigmaQ_jrk_ = 1;
  double alphaQ_jrk_ = 0.5;

  // Standard deviation for the measurement covariance
  double sigmaR_pos_ = 1.0;
  double sigmaR_vel_ = 0.1;
  double sigmaR_range_ = 0.6;
  double sigmaR_zenith_ = 0.01745;

  bool kalman_on_{true};
  bool radar_meas_{false};
  Eigen::Vector2d radarPos_;

};


} // namespace quadrotor
