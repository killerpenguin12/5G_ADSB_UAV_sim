#pragma once

#include <fstream>

#include "common_cpp/common.h"
#include "common_cpp/logger.h"
// #include "pb_vi_ekf/ekf.h"
// #include "quad_control.h"
// #include "sensors.h"
#include "vehicle.h"
#include "environment.h"
#include "geometry/support.h"
#include <math.h>


using namespace Eigen;


namespace quadrotor
{


class Quadrotor
{

public:

  Quadrotor();
  Quadrotor(const std::string &filename,
    std::string name,
    std::vector<double> loaded_wps,
    const environment::Environment& env,
    const bool &use_random_seed,
    const int& id);
  ~Quadrotor();

  void load(const std::string &filename,
    std::string name,
    std::vector<double> loaded_wps,
    const environment::Environment &env,
    const bool &use_random_seed);
  void run(const double &t, const environment::Environment& env, uVector u);
  //MSO stuff here
  void initMSO(int frames,int vehicle_identifers);
  int getLSBs(int val);
  int pseudo_random_number(int val);
  int full_mso_range();
  int restricted_mso_range();
  std::vector<long> extract_lsbs();
  void updateMSO();
  long encode_lsbs(double value);
  std::vector<double> get_lat_lon_alt();
  std::vector<int> transmissions (int t = 0,int u = 0);
  std::vector<int> get_dist();
  std::vector<std::pair<int,int>> distanceDict;
  std::vector<float> recieved_powerDict;
  std::vector<bool> successful_decodeDict;
  void saveTransmission(std::vector<std::pair<int,int>> distanceDict,std::vector<float> recieved_powerDict,std::vector<bool> successful_decodeDict){
    distanceDict = distanceDict;
    recieved_powerDict = recieved_powerDict;
    successful_decodeDict = successful_decodeDict;
  };
  int getMSO(){return message_start_opportunity;};
  void increaseFrame(){frame += 1;};
  bool getMessageSuccess(){return messageSuccess;};
  void setMessageSuccess(bool val){messageSuccess = val;};
  //End of MSO stuff
  const vehicle::Stated& getState2() { //was const 
    std::vector<double> one;
    if (x_.p(0) < 0)
    {
      one.push_back(ceil(x_.p(0)));
      one.push_back(floor(x_.p(1)));
      one.push_back(floor(x_.p(2)));
    }
    else{
      one.push_back(floor(x_.p(0)));
      one.push_back(ceil(x_.p(1)));
      one.push_back(floor(x_.p(2)));
    }
    vehicle::Stated x0_;
    x0_ = vehicle::Stated(x_);
    x0_.p = Vector3d(one[0], one[1], one[2]);
    x_ = x0_;
    return x_; 
    }
  const vehicle::Stated& getState() const{return x_; }
  int id_;
  std::string name_;

  void f(const vehicle::Stated& x, const uVector& u,
    const Eigen::Vector3d& vw, vehicle::dxVector& dx);
private:

  void propagate(const double &dt, const uVector& u, const Eigen::Vector3d& vw);
  void updateAccels(const uVector& u, const Eigen::Vector3d& vw);
  void getOtherVehicles(const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d> >& all_vehicle_positions,
                        const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d> >& all_vehicle_velocities);
  void log(const double &t);
  // void runEstimator(const double &t, const sensors::Sensors &sensors, const Vector3d &vw, const vehicle::Stated &x_t, const MatrixXd &lm);
  // vehicle::Stated getControlStateFromEstimator() const;

  // Controller controller_;
  // sensors::Sensors sensors_;
  // pbviekf::EKF estimator_;

  vehicle::Stated x_;
  vehicle::dxVector dx_;
  uVector u_;

  bool accurate_integration_, control_using_estimates_;
  double mass_, max_thrust_, t_prev_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  Eigen::Matrix3d linear_drag_matrix_;
  Eigen::Matrix3d angular_drag_matrix_;
  Eigen::Vector3d v_rel_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_positions_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_velocities_;

  common::Logger state_log_;
  common::Logger euler_log_;
  bool messageSuccess = 0;
  //MSO stuff
  long lat = 0;
  long lon = 0;
  long alt = 0;
  long latitude_lsbs = 0;  // 12 L.S.B.'s of the most recent valid "LATITUDE"
  long longitude_lsbs = 0;  // 12 L.S.B.'s of the most recent valid "LONGITUDE"
  int frame = 0;   // current UAT frame
  int previous_random_number = 0;  // R(m-1) most recent random number chosen
  int frame_before_restricted = 0;  // the frame just prior to entering the restricted MSO range mode
  //Stated x_ contains the position
  int vehicle_identifier = 0; //which vehicle we are
  int message_start_opportunity = 0; //our mso
  

};


} // namespace quadrotor
