#include "quadrotor.h"
#include "GeodeticConverter.h"
#include "math.h"
#include <iomanip>

namespace quadrotor
{


Quadrotor::Quadrotor()  : t_prev_(0.0) {}


Quadrotor::Quadrotor(const std::string &filename,
  std::string name,
  std::vector<double> loaded_wps,
  const environment::Environment& env,
  const bool& use_random_seed,
  const int& id)
  : t_prev_(0.0), id_(id)
{
  load(filename,
  name, loaded_wps, env, use_random_seed);
}


Quadrotor::~Quadrotor() {}


void Quadrotor::load(const std::string &filename,
  std::string name,
  std::vector<double> loaded_wps,
  const environment::Environment& env,
  const bool& use_random_seed)
{
  // Instantiate Sensors, Controller, and Estimator classes
  name_ = name;
  // controller_.load(filename, loaded_wps, use_random_seed, name, vo_client);
  // sensors_.load(filename, use_random_seed, name_);
  // estimator_.load("../params/pb_vi_ekf_params.yaml", name_);

  // Load all Quadrotor parameters
  accurate_integration_ = true;
  mass_ = 5.0;
  max_thrust_ = 98.0665;
  //max_thrust_ = 500.000;
  control_using_estimates_ = true;
  // common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  // common::get_yaml_node("mass", filename, mass_);
  // common::get_yaml_node("max_thrust", filename, max_thrust_);
  // common::get_yaml_node("control_using_estimates", filename, control_using_estimates_);
  vehicle::xVector x0;
  x_.p[0] = 0;
  x_.p[1] = 0;
  updateMSO();
  //vehicle::xVector x0(-10, 0, 0,0, 0, 0,0, 0, 0,1, 0, 0, 0,0, 0, 0,0, 0, 0);
  // x0 << -10, 0, 0,    //# POS
  //    0, 0, 0,    //# VEL
  //    0, 0, 0,    //# LIN ACC
  //    1, 0, 0, 0, //# ATT
  //    0, 0, 0,    //# OMEGA
  //    0, 0, 0;
  // vehicle::Stated
  //   x0.State(-10, 0, 0,    //# POS
  //    0, 0, 0,    //# VEL
  //    0, 0, 0,    //# LIN ACC
  //    1, 0, 0, 0, //# ATT
  //    0, 0, 0,    //# OMEGA
  //    0, 0, 0);    //# ANG ACC

  common::get_yaml_eigen("x0", filename, x0);
  // inertia_matrix_.row(0) << 0.6271, 0.6271, 1.25;
  // linear_drag_matrix_.row(0) << 0.1, 0.1, 0.001;
  // angular_drag_matrix_.row(0) << 0.001, 0.001, 0.001;

  common::get_yaml_eigen_diag("inertia", filename, inertia_matrix_);
  common::get_yaml_eigen_diag("linear_drag", filename, linear_drag_matrix_);
  common::get_yaml_eigen_diag("angular_drag", filename, angular_drag_matrix_);
  x_ = vehicle::Stated(x0);
  x_.p = Vector3d(loaded_wps[0], loaded_wps[1], loaded_wps[2]); //this is our position, x_.p
  //std::cout << loaded_wps[0] << "Here it was" << std::endl;
  x_.v = -x_.p.normalized(); //* 5.0; NOW you know
  x_.drag = linear_drag_matrix_(0,0);
  inertia_inv_ = inertia_matrix_.inverse();

  // Randomly initialize estimator vel/roll/pitch/drag
  bool random_init;
  double v0_err, roll0_err, pitch0_err, drag0_err;
  random_init = true;
  v0_err = 0.05;
  roll0_err = 0.0524;
  pitch0_err = 0.0524;
  drag0_err = 0.01;
  // common::get_yaml_node("ekf_random_init", filename, random_init);
  // common::get_yaml_node("ekf_v0_err", filename, v0_err);
  // common::get_yaml_node("ekf_roll0_err", filename, roll0_err);
  // common::get_yaml_node("ekf_pitch0_err", filename, pitch0_err);
  // common::get_yaml_node("ekf_drag0_err", filename, drag0_err);
  if (random_init)
  {
    double roll_new = x_.q.roll() + roll0_err * Vector1d::Random()(0);
    double pitch_new = x_.q.pitch() + pitch0_err * Vector1d::Random()(0);
    // estimator_.setVelocity(x_.v + v0_err * Vector3d::Random());
    // estimator_.setAttitude(quat::Quatd(roll_new,pitch_new,x_.q.yaw()).elements());
    // estimator_.setDrag(x_.drag + drag0_err * Vector1d::Random()(0));
  }

  // Initialize other classes
  // controller_.computeControl(getState(), 0, u_, other_vehicle_positions_, other_vehicle_velocities_);
  updateAccels(u_, env.get_vw());
  // sensors_.updateMeasurements(0, x_, env.get_vw(), env.get_points());
  // runEstimator(0, sensors_, env.get_vw(), getState(), env.get_points());

  // Initialize loggers and log initial data
  //std::stringstream ss_s, ss_e;
  //ss_s << "/tmp/" << name_ << "_true_state.log";
  //ss_e << "/tmp/" << name_ << "_euler_angles.log";
  //state_log_.open(ss_s.str());
  //euler_log_.open(ss_e.str());
  //log(0);
}


void Quadrotor::run(const double &t, const environment::Environment& env, uVector u)
{
  u_ = u;
  //std::cout << "u: " << u << std::endl;
  getOtherVehicles(env.getVehiclePositions(), env.getVehicleVelocities());
  propagate(t, u_, env.get_vw()); // Propagate truth to current time step
  // if (control_using_estimates_)
  //   controller_.computeControl(getControlStateFromEstimator(), t, u_, other_vehicle_positions_[0]);
  // else
    // controller_.computeControl(getState(), t, u_, other_vehicle_positions_, other_vehicle_velocities_);
  updateAccels(u_, env.get_vw()); // Update true acceleration
  // sensors_.updateMeasurements(t, x_, env.get_vw(), env.get_points());
  // runEstimator(t, sensors_, env.get_vw(), getState(), env.get_points());
  log(t); // Log current data
}


void Quadrotor::f(const vehicle::Stated& x, const uVector& u,
                  const Vector3d& vw, vehicle::dxVector& dx)
{
  v_rel_ = x.v - x.q.rotp(vw);
  dx.segment<3>(vehicle::DP) = x.q.rota(x.v);
  dx.segment<3>(vehicle::DV) = -common::e3 * u(THRUST) * max_thrust_ / mass_ - linear_drag_matrix_ * v_rel_ +
                                 common::gravity * x.q.rotp(common::e3) - x.omega.cross(x.v);
  dx.segment<3>(vehicle::DQ) = x.omega;
  dx.segment<3>(vehicle::DW) = inertia_inv_ * (u.segment<3>(TAUX) - x.omega.cross(inertia_matrix_ * x.omega) -
                                angular_drag_matrix_ * x.omega);

  // std::cout << "Force: \n";
  // std::cout << "dx: " << dx << std::endl;
  // std::cout << x.v << std::endl;
  // std::cout << x.q << std::endl;
  // std::cout << x.q.rota(x.v)<< std::endl;
  // std::cout << -common::e3 * u(THRUST) * max_thrust_ / mass_ << std::endl;
  // std::cout << linear_drag_matrix_ << std::endl;
  // std::cout << v_rel_ << std::endl;
}


void Quadrotor::propagate(const double &t, const uVector& u, const Vector3d& vw)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;
  //std::cout << "t_prev_: " << t_prev_ << std::endl;
  // Integration
  if (accurate_integration_)
  {
    //std::cout << "RK4!!!" << std::endl;
    // 4th order Runge-Kutta
    /*std::cout << "dt: "  << dt << std::endl;
  std::cout << "x_: "  << x_.p << std::endl;
  std::cout << "u: "  << u << std::endl;
  std::cout << "vw: "  << vw << std::endl;
  std::cout << "dx_: "  << dx_ << std::endl;*/
    vehicle::rk4<COMMAND_SIZE>(std::bind(&Quadrotor::f, this,
                               std::placeholders::_1,std::placeholders::_2,
                               std::placeholders::_3,std::placeholders::_4),
                               dt, x_, u, vw, dx_);
  /*std::cout << "dt: "  << dt << std::endl;
  std::cout << "x_: "  << x_.p << std::endl;
  std::cout << "u: "  << u << std::endl;
  std::cout << "vw: "  << vw << std::endl;
  std::cout << "dx_: "  << dx_ << std::endl;
  std::cout << "###########################" << std::endl;*/
  }
  else
  {
    //std::cout << "Euler" << std::endl;
    // Euler
    f(x_, u, vw, dx_);
    dx_ *= dt;
  }
  x_ += dx_; //So here we need to change this to change our speed.
}


void Quadrotor::updateAccels(const uVector &u, const Vector3d &vw)
{
  static vehicle::dxVector dx;
  f(x_, u, vw, dx);
  x_.lin_accel = dx.segment<3>(vehicle::DV);
  x_.ang_accel = dx.segment<3>(vehicle::DW);
}


void Quadrotor::log(const double &t)
{
  // Write data to binary files and plot in another program
  //state_log_.log(t);
  //state_log_.logMatrix(x_.toEigen());
  //euler_log_.log(t);
  //euler_log_.logMatrix(x_.q.euler());
}


void Quadrotor::getOtherVehicles(const std::vector<Vector3d, aligned_allocator<Vector3d> > &all_vehicle_positions,
                                 const std::vector<Vector3d, aligned_allocator<Vector3d> > &all_vehicle_velocities)
{
  other_vehicle_positions_.clear();
  other_vehicle_velocities_.clear();

  for (int i = 0; i < all_vehicle_positions.size(); ++i)
  {
    if (i != id_)
    {
      other_vehicle_positions_.push_back(all_vehicle_positions[i]);
      other_vehicle_velocities_.push_back(all_vehicle_velocities[i]);
    }
  }

}


// void Quadrotor::runEstimator(const double &t, const sensors::Sensors &sensors, const Vector3d& vw, const vehicle::Stated& xt, const MatrixXd& lm)
// {
//   // Run all sensor callbacks
//   if (sensors.new_imu_meas_)
//   {
//     estimator_.imuCallback(sensors.imu_);
//     if (estimator_.getFilterUpdateStatus())
//       estimator_.logTruth(t, xt.p, xt.v, xt.q, sensors.getAccelBias(), sensors.getGyroBias(), xt.drag, xt.omega, lm);
//   }
//   if (sensors.new_camera_meas_)
//     estimator_.cameraCallback(sensors.image_);
//   if (sensors.new_gps_meas_)
//     estimator_.gpsCallback(sensors.gps_);
//   if (sensors.new_mocap_meas_)
//     estimator_.mocapCallback(sensors.mocap_);
// }
//
//
// vehicle::Stated Quadrotor::getControlStateFromEstimator() const
// {
//   vehicle::Stated state;
//   state.p = estimator_.getGlobalPosition();
//   state.q = estimator_.getGlobalAttitude();
//   state.v = estimator_.getState().v;
//   return state;
// }
//#########################################################################################
//Coulton Added from here: adding MSO stuff
//Gives us our first mso, starting off. This was in the main code before, but I feel like it fits better here.
void Quadrotor::initMSO(int frames, int vehicle_identifiers)
{
  frame = frames;
  //std::cout << "Our frame: " << frame << std::endl;
  vehicle_identifier = vehicle_identifiers;
  //std::cout << "Our vehicle: " << vehicle_identifier << std::endl;
}

int Quadrotor::getLSBs(int val)
{
    if(val == 0)
    {
          //std::cout << "LSB 0 " << latitude_lsbs <<std::endl;
          return latitude_lsbs;  // latitude
    }
    else if (val == 1)
    {
          //std::cout << "LSB 1 " << longitude_lsbs <<std::endl;
          return longitude_lsbs;  // longitude
    }
    else
    {
          std::cout << "ERROR: frame is invalid in get_least_significant_bits" << std::endl;
          return 0;
    }
}
int Quadrotor::pseudo_random_number(int val)
{
  //std::cout <<"Frame: " << frame << std::endl;
   if(val == 0)
   {
     previous_random_number = getLSBs(0) % 3200;
     //std::cout << "val is 0 and previous is: " << previous_random_number << std::endl;
     return previous_random_number;
   }
   else if(val >= 1)
   {
     //calc pseudo-random number
     //std::cout << "previous random number: " << previous_random_number <<std::endl;
     previous_random_number = (4001 * previous_random_number + getLSBs(val % 2)) % 3200;
     //std::cout << "frame is not 0 and previous is: " << previous_random_number << std::endl;
     return previous_random_number;
   }
   else
   {
      std::cout << "ERROR: frame is invalid in pseudo_random_number" << std::endl;
      return 0;
   }
}
int Quadrotor::full_mso_range()
{
  //std::cout << "X: " << x_.p[0] << " Y: " << x_.p[1] << std::endl;
  //std::cout << "Frame before restricted: " << frame_before_restricted << std::endl;
  frame_before_restricted = frame; //update k
  //std::cout << "Frame: " << frame << std::endl;
  //std::cout << " #########################################################" << std::endl;
  //std::cout << "Position: " << x_.p[1] << " " << x_.p[0] << " " << x_.p[2] << std::endl;
  // std::cout << "VEHICLE PRINTING" << std::endl;
  // std::cout << "##################################################" << std::endl;
  // std::cout << "Frame: " << frame << std::endl;
  // std::cout << "Position: " << x_.p[1] << ", " << x_.p[0] << ", " << x_.p[2] << std::endl;
  // std::cout << "Random Number: " << pseudo_random_number(frame) << std::endl;
  message_start_opportunity = (752 + pseudo_random_number(frame));  // calculate MSO
  // std::cout << "mso: " << message_start_opportunity << std::endl;
  // std::cout << "lat: " << latitude_lsbs << std::endl;
  // std::cout << "long: " << longitude_lsbs << std::endl;
  // std::cout << "END OF VEHICLE PRINTING" << std::endl;
  // std::ofstream file("LSB_Test.csv" , std::ios::app );
  // file << latitude_lsbs << "," << longitude_lsbs << "," << frame << "," << x_.p[1] << "," << x_.p[0] << "," << message_start_opportunity << "\n";
  //std::cout << std::endl;
  // std::ofstream help("Positions.csv" , std::ios::app );
  // help << vehicle_identifier << "," <<x_.p[1] << "," << x_.p[0] << "\n";
  //std::cout << "MSO(full_mso): " << message_start_opportunity << std::endl; 
  //std::cout << "Random number: " << pseudo_random_number(frame) << std::endl;
  // std::cout << "Previous Random Numbers: " << previous_random_number << std::endl;
  // std::cout << "Frame: " << frame << std::endl; 
  // std::cout << "MSO: " << message_start_opportunity << std::endl;
  // std::cout << "Lat lsbs: " << latitude_lsbs << std::endl;
  // std::cout << "Long lsbs: " << longitude_lsbs << std::endl;
  // std::cout << " #########################################################" << std::endl;

  return message_start_opportunity;  // return MSO
}
int Quadrotor::restricted_mso_range()
{
  int r_star = pseudo_random_number(frame_before_restricted) - (
                pseudo_random_number(frame_before_restricted) % 800);  // definition of R_star (R*)
  message_start_opportunity = (752 + r_star + (pseudo_random_number(frame) % 800));  // calculate MSO
  return message_start_opportunity;  // return MSO
}
// Converts NED coordinate to geodetic, based on a relative geodetic coordinate
std::vector<double> Quadrotor::get_lat_lon_alt()
{
    //std::cout << "Positons for digits: " << x_.p[0] << " " << x_.p[1] << std::endl;
    float xPos = x_.p[0];
    float yPos = x_.p[1];
    // std::ofstream pos("Positions1.csv" , std::ios::app );
    // pos << id_ << "," << x_.p[0] << "," << x_.p[1] << '\n';
    // pos.close();
    float zPos = 100.0;//x_.p[2];

    double initial_lat = 40.2338;
    double initial_lon = -111.6585;
    double initial_alt = 100.0;
    // double *alt_ = &initial_alt;
    // double *lon_ = &initial_lon;
    // double *lat_ = &initial_lat;
    double alt_saved = (double) alt;
    double lat_saved = (double) lat;
    double lon_saved = (double) lon;
    double *alt_ = &alt_saved;
    double *lon_ = &lon_saved;
    double *lat_ = &lat_saved;
    GeodeticConverter(initial_lat,initial_lon,initial_alt).ned2Geodetic(yPos,xPos,zPos,lat_,lon_,alt_);
    std::vector<double> final (0,0);
    final.push_back(*lat_);
    final.push_back(*lon_);
    final.push_back(*alt_);
    return final;
}

std::vector<long> Quadrotor::extract_lsbs()
{
    // float xPos = x_.p[0];
    // float yPos = x_.p[1];
    // float zPos = x_.p[2];
    std::vector<double> coordinate (0,0);
    coordinate = get_lat_lon_alt();
    //latitude_lsbs;
    //longitude_lsbs;
    std::vector<long> final_val (0,0);
    //std::cout << "Before modification lat: " << latitude_lsbs << std::endl;
    latitude_lsbs = encode_lsbs(coordinate[0]);
    //std::cout << "After modification lat: " << latitude_lsbs << std::endl;
    longitude_lsbs = encode_lsbs(coordinate[1]);
    //std::cout << "After modification long: " << longitude_lsbs << std::endl;
    final_val.push_back(latitude_lsbs);
    final_val.push_back(longitude_lsbs);
    final_val.push_back(100);//100 is our altitude. 
    // std::cout << "VEHICLE PRINTING" << std::endl;
    // std::cout << "Coord0: " << std::setprecision (25) << coordinate[0] << std::endl;
    // std::cout << "Coord1: " << std::setprecision (25) << coordinate[1] << std::endl;
    // std::cout << "latitde_lsbs: " << latitude_lsbs<< std::endl;
    // std::cout << "Long_lsbs: " << longitude_lsbs << std::endl;
    // std::cout << " " << std::endl;

    return final_val;
}
//This is just to get our 12 lsbs.
std::string toBinary(int n)
{
    std::string r;
    while(n!=0) {r=(n%2==0 ?"0":"1")+r; n/=2;}
    return r;
}
long int binaryToDecimal(long n)
{
    long num = n;
    long int dec_value = 0;
 
    // Initializing base value to 1, i.e 2^0
    int base = 1;
 
    long temp = num;
    while (temp) {
        long last_digit = temp % 10;
        temp = temp / 10;
 
        dec_value += last_digit * base;
 
        base = base * 2;
    }
    //float final = (float) dec_value;
 
    return dec_value;
}
long Quadrotor::encode_lsbs(double value)
{
    //long stuff
    double binary_value = value;
    long int final_val;
    if(binary_value < 0)
    {
            binary_value += 180;  // negative values (South for LAT, West for LON)
            // have slightly different encoding than positive
    }
    double least_significant_bit = 360.0/(pow(2.0,24.0));
    binary_value = binary_value / least_significant_bit; // LSB is a conversion factor in D0-282B. 360/(2^24)
    binary_value = std::round(binary_value);  // round to the nearest integer to prep for binary conversion
    int binary = (int) binary_value; // change to an int for binary conversion, otherwise the format is weird
    //std::cout << "int binary: " << binary << std::endl;
    std::string b = toBinary(binary);
    //std::cout << "b: " << b << std::endl;
    std::string a;
    for(int i = b.size() - 12; i < b.size(); i++)
      a += b[i]; //get our lsbs
    //std::cout << "a: " << a << std::endl;
    final_val = binaryToDecimal(std::stol(a)); // turns the 12 bit binary string into an double value
    //std::cout << "Final value: " << final_val << std::endl;
    return final_val;  // returns the correctly encoded 12 L.S.B.s of the value
}
std::vector<int> Quadrotor::get_dist() //Dont use this, for some reasosn it only detects num_vehicles - 2. Not sure why.
{ 
  //std::cout << "other vehicles: " << other_vehicle_positions_.size() << std::endl;
  //For some reason this only has 8 other vehicles when we have 10 total vehicles. Unknown as to why.
  std::vector<int> dist;
  for(int i = 0; i < other_vehicle_positions_.size(); i++)
  {
  double x = other_vehicle_positions_[i][0] - x_.p[0];
  double y = other_vehicle_positions_[i][1] - x_.p[1];
  double distance;
  distance = pow(x,2) + pow(y,2);
  distance = sqrt(distance);
  dist.push_back(distance);
  //std::cout << "distance: " << distance << std::endl;
  //std::cout << "dist i: " << dist[i] << std::endl;
  }
  //getOtherVehicles(env.getVehiclePositions(), env.getVehicleVelocities()); //dont use assume values are current


  return dist;
}

void Quadrotor::updateMSO() //Dont really need this, position si already updated in the program and we just use the premade updater.
{
  //std::cout << "Does this run? " << std::endl;
  std::vector <long> extracted(0);
  extracted = extract_lsbs();
  lat = extracted[0];
  lon = extracted[1];
  alt = extracted[2];
  // std::cout << "VEHICLE PRINTING" << std::endl;
  // std::cout << "lat: " << lat << std::endl;
  // std::cout << "long: " << lon << std::endl;
}

} // namespace quadrotor
