#include "quad_control.h"


namespace quadrotor
{


Controller::Controller() :
  prev_time_(0)
{
  dhat_.setZero();
}


Controller::~Controller()
{
  //command_state_log_.close();
  //command_log_.close();
  //euler_command_log_.close();
  //target_log_.close();
}


void Controller::load(const std::string& filename,
  std::vector<double> loaded_wps,
  const bool& use_random_seed,
  const std::string& name,
  double max_vel)
{

  // Initialize random number generator
  int seed;
  if (use_random_seed)
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  else
    seed = 0;
  rng_ = std::default_random_engine(seed);
  srand(seed);
  max_thrust_ = 98.0665;
  mass_ = 5.0;
  throttle_eq_ = 0.5;
  //common::get_yaml_node("throttle_eq", filename, throttle_eq_);
  //common::get_yaml_node("mass", filename, mass_);
  //common::get_yaml_node("max_thrust", filename, max_thrust_);

  Vector3d Kp_diag(0.15,0.15,0.15), Kd_diag(0.0,0.0,0.0), Kv_diag(1.5,1.5,1.5);
  //Kp: [0.15, 0.15, 0.15]
  //Kd: [0.0, 0.0, 0.0]
  //Kv: [1.5, 1.5, 1.5]
  if (common::get_yaml_eigen("Kp", filename, Kp_diag))
    K_p_ = Kp_diag.asDiagonal();
  if (common::get_yaml_eigen("Kd", filename, Kd_diag))
    K_d_ = Kd_diag.asDiagonal();
  if (common::get_yaml_eigen("Kv", filename, Kv_diag))
    K_v_ = Kv_diag.asDiagonal();
  // roll_.kp_ = 10.0;
  // roll_.ki_ = 0.0;
  // roll_.kd_ = 5.0;
  // pitch_.kp_ = 10.0;
  // pitch_.ki_ = 0.0;
  // pitch_.kd_ = 5.0;
  // yaw_rate_.kp_ = 10.0;
  // yaw_rate_.ki_ = 0.0;
  // yaw_rate_.kd_ = 0.0;
  // roll_.max_ = 200.0;
  // pitch_.max_ = 200.0;
  // yaw_rate_.max_ = 50.0;
  // max_.roll = 1.0;
  // max_.pitch = 1.0;
  // max_.yaw_rate = 1.5;
  // max_.throttle = 1.0;
  
  common::get_yaml_node("roll_kp", filename, roll_.kp_);
  common::get_yaml_node("roll_ki", filename, roll_.ki_);
  common::get_yaml_node("roll_kd", filename, roll_.kd_);
  common::get_yaml_node("pitch_kp", filename, pitch_.kp_);
  common::get_yaml_node("pitch_ki", filename, pitch_.ki_);
  common::get_yaml_node("pitch_kd", filename, pitch_.kd_);
  common::get_yaml_node("yaw_rate_kp", filename, yaw_rate_.kp_);
  common::get_yaml_node("yaw_rate_ki", filename, yaw_rate_.ki_);
  common::get_yaml_node("yaw_rate_kd", filename, yaw_rate_.kd_);
  common::get_yaml_node("max_tau_x", filename, roll_.max_);
  common::get_yaml_node("max_tau_y", filename, pitch_.max_);
  common::get_yaml_node("max_tau_z", filename, yaw_rate_.max_);
  common::get_yaml_node("max_roll", filename, max_.roll);
  common::get_yaml_node("max_pitch", filename, max_.pitch);
  common::get_yaml_node("max_yaw_rate", filename, max_.yaw_rate);
  common::get_yaml_node("max_throttle", filename, max_.throttle);
  if(max_vel == 0.0)
    //max_.vel = 5;
    common::get_yaml_node("max_vel", filename, max_.vel);
  else
    max_.vel = max_vel;

  //path_type_ = 0;
  common::get_yaml_node("path_type", filename, path_type_);


  // int num_waypoints = std::floor(loaded_wps.size()/4.0);
  // waypoints_ = Map<MatrixXd>(loaded_wps.data(), 4, num_waypoints);
  // current_waypoint_id_ = 0;
  // std::cout << "Waypionts Loaded: " << waypoints_ << std::endl;
  //
  // common::get_yaml_node("waypoint_threshold", filename, waypoint_threshold_);
  // common::get_yaml_node("waypoint_velocity_threshold", filename, waypoint_velocity_threshold_);
  
  // double traj_north_period, traj_east_period, traj_alt_period, traj_yaw_period;
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
  // traj_north_freq_ = 2.0 * M_PI / traj_north_period;
  // traj_east_freq_ = 2.0 * M_PI / traj_east_period;
  // traj_alt_freq_ = 2.0 * M_PI / traj_alt_period;
  // traj_yaw_freq_ = 2.0 * M_PI / traj_yaw_period;
  
  // common::get_yaml_node("circ_rd", filename, circ_rd_);
  // common::get_yaml_node("circ_hd", filename, circ_hd_);
  // common::get_yaml_node("circ_kr", filename, circ_kr_);
  // common::get_yaml_node("circ_kp", filename, circ_kp_);
  // common::get_yaml_node("circ_kh", filename, circ_kh_);

  // double target_noise_stdev;
  // Vector3d z_(5, 0, 0);
  // kvz_ = 1.0;
  // Vector3d vz_(0, 0, 0);
  // kz_ = 1.0;
  // bearing_only_ = true;
  // use_target_truth_ = false;
  double target_noise_stdev;
  common::get_yaml_node("target_gain", filename, kz_);
  common::get_yaml_node("target_velocity_gain", filename, kvz_);
  common::get_yaml_eigen<Vector3d>("target_z0", filename, z_);
  common::get_yaml_eigen<Vector3d>("target_vz0", filename, vz_);
  common::get_yaml_node("bearing_only", filename, bearing_only_);
  common::get_yaml_node("use_target_truth", filename, use_target_truth_);
  common::get_yaml_node("target_noise_stdev", filename, target_noise_stdev);
  target_noise_dist_ = std::normal_distribution<double>(0.0, target_noise_stdev);
  target_noise_.setZero();

  // Initialize loggers
  // std::stringstream ss_cs, ss_c, ss_ec, ss_t;
  // ss_cs << "/tmp/" << name << "_commanded_state.log";
  // ss_c << "/tmp/" << name << "_command.log";
  // ss_ec << "/tmp/" << name << "_euler_command.log";
  // ss_t << "/tmp/" << name << "_target.log";
  // command_state_log_.open(ss_cs.str());
  // command_log_.open(ss_c.str());
  // euler_command_log_.open(ss_ec.str());
  // target_log_.open(ss_t.str());
}

quadrotor::uVector Controller::computeControl(const vehicle::Stated &x,
    const double t,
    vehicle::Stated xc)
{
  uVector u;
  xc_ = xc;
  // Unpack useful states
  double phi = x.q.roll();
  double theta = x.q.pitch();
  double psi = x.q.yaw();

  double dt = t - prev_time_;
  prev_time_ = t;

  if (dt < 0.0001)
  {
    u.setZero();
    return u;
  }

  double throttle;
  if (path_type_ == 0 || path_type_ == 1)
  {
    // Refresh the waypoint or trajectory
    // if (path_type_ == 0)
    //   updateWaypointManager();
    // else
    //   updateTrajectoryManager(t);

    // get data that applies to both position and velocity control
    Matrix3d R_v_to_v1 = common::R_v_to_v1(psi); // rotation from vehicle to vehicle-1 frame
    Matrix3d R_v1_to_b = common::R_v_to_b(phi, theta, 0.0); // rotation from vehicle-1 to body frame

    // xc_.v = R_v_to_v1 * K_p_ * (xc_.p - x.p);


    // get yaw rate direction and allow it to saturate
    xc_.omega(2) = common::wrapAngle(xc_.q.yaw() - x.q.yaw(), M_PI); // wrap angle

    // get velocity command and enforce max velocity
    double vmag = xc_.v.norm();
    if (vmag > max_.vel)
      xc_.v = xc_.v * max_.vel / vmag;

    Vector3d vhat = R_v1_to_b.transpose() * x.v; // vehicle-1 velocity estimate
    Vector3d k_tilde;

    dhat_ = dhat_ - K_d_*(xc_.v - vhat)*dt; // update disturbance estimate
    k_tilde = throttle_eq_ * (common::e3 - (1.0 / common::gravity) * (K_v_ * (xc_.v - vhat) - dhat_));


    // pack up throttle command
    throttle = common::e3.transpose() * R_v1_to_b * k_tilde;

    Vector3d kd = (1.0 / throttle) * k_tilde; // desired body z direction
    kd = kd / kd.norm(); // need direction only
    double tilt_angle = acos(common::e3.transpose() * kd); // desired tilt

    // get shortest rotation to desired tilt
    quat::Quatd q_c;
    if (tilt_angle > 1e-6)
    {
      Vector3d k_cross_kd = common::e3.cross(kd);
      q_c = quat::Quatd::exp(tilt_angle * k_cross_kd / k_cross_kd.norm());
    }

    // pack up attitude commands
    xc_.q = quat::Quatd(q_c.roll(), q_c.pitch(), xc_.q.yaw());
  }
  // else if (path_type_ == 2)
  // {
  //   // Create noisy bearing measurement of target and full vector measurement
  //   if (!use_target_truth_)
  //     common::randomNormal(target_noise_, target_noise_dist_, rng_);
  //   Vector3d z_true = x.q.rotp(pt - x.p);
  //   Vector3d z = z_true + target_noise_;
  //   Vector3d ez = z / z.norm();
  //   Vector3d z_tilde = z_ - z;
  //
  //   // Target estimator
  //   Vector3d zdot(0, 0, 0), vzdot(0, 0, 0);
  //   if (bearing_only_)
  //   {
  //     zdot = -x.v - x.omega.cross(z_) - kz_ * (common::I_3x3 - ez * ez.transpose()) * z_;
  //   }
  //   else
  //   {
  //     zdot = vz_ - x.omega.cross(z_) - kz_ * z_tilde;
  //     vzdot = -x.omega.cross(vz_) - x.lin_accel - x.omega.cross(x.v) - kvz_ * z_tilde;
  //   }
  //   z_ += zdot * dt;
  //   vz_ += vzdot * dt;
  //
  //
  //   // Extract local level frame rotation
  //   quat::Quatd q_l2b(phi, theta, 0.0);
  //
  //   // Commanded velocity in the local level reference frame
  //   static const Matrix3d IPe3 = common::I_3x3 - common::e3 * common::e3.transpose();
  //   static const Matrix3d Pe3 = common::e3 * common::e3.transpose();
  //   double r = (IPe3 * q_l2b.rota(z_)).norm();
  //   double h = (Pe3 * q_l2b.rota(z_)).norm();
  //
  //   Vector3d er = IPe3 * q_l2b.rota(ez);
  //   er /= er.norm();
  //   Vector3d ep = common::e3.cross(er);
  //   ep /= ep.norm();
  //
  //   double r_tilde = r - circ_rd_;
  //   double h_tilde = h - circ_hd_;
  //   if (bearing_only_)
  //     xc_.v = circ_kr_ * r_tilde * er + circ_kp_ * ep + circ_kh_ * h_tilde * common::e3;
  //   else
  //     xc_.v = circ_kr_ * r_tilde * er + circ_kp_ * ep + circ_kh_ * h_tilde * common::e3 + q_l2b.rota(vz_ + x.v);
  //   double vmag = xc_.v.norm();
  //   if (vmag > max_.vel)
  //     xc_.v *= max_.vel / vmag;
  //
  //   // Commanded yaw rate
  //   xc_.omega(2) = 10.0 * common::e3.transpose() * common::e1.cross(ez);
  //
  //   Vector3d vtilde = xc_.v - q_l2b.rota(x.v);
  //   Vector3d omega_l = common::e3 * common::e3.transpose() * q_l2b.rota(x.omega);
  //   Vector3d vl = q_l2b.rota(x.v);
  //
  //   // Commanded throttle
  //   throttle = throttle_eq_ * common::e3.transpose() * q_l2b.rotp(common::e3
  //              - 1.0 / common::gravity * (omega_l.cross(vl) + K_v_ * vtilde));
  //
  //   // Commanded body axes in the inertial frame
  //   Vector3d kbc = common::e3 - 1.0 / common::gravity * (omega_l.cross(vl) + K_v_ * vtilde);
  //   kbc /= kbc.norm();
  //   Vector3d jbc = kbc.cross(q_l2b.rota(ez));
  //   jbc /= jbc.norm();
  //   Vector3d ibc = jbc.cross(kbc);
  //   ibc /= ibc.norm();
  //
  //   // Build commanded attitude
  //   Matrix3d Rc; // Should be inertial to body rotation
  //   Rc << ibc, jbc, kbc;
  //
  //   // Extract roll and pitch angles
  //   double phi_c = atan2(-Rc(1,2), Rc(1,1));
  //   double theta_c = atan2(-Rc(2,0),Rc(0,0));
  //   xc_.q = quat::Quatd(phi_c, theta_c, xc_.q.yaw());
  // }
  else
    throw std::runtime_error("Undefined path type in quadrotor controller.");

  // Create Roll and Pitch Command
  double phi_c = xc_.q.roll();
  double theta_c = xc_.q.pitch();
  double r_c = xc_.omega(2);
  throttle = common::saturate(throttle, max_.throttle, 0.0);
  phi_c = common::saturate(phi_c, max_.roll, -max_.roll);
  theta_c = common::saturate(theta_c, max_.pitch, -max_.pitch);
  r_c = common::saturate(r_c, max_.yaw_rate, -max_.yaw_rate);

  // Calculate the Final Output Torques using PID
  u(quadrotor::THRUST) = throttle;
  u(quadrotor::TAUX) = roll_.run(dt, phi, phi_c, false, x.omega(0), true);
  u(quadrotor::TAUY) = pitch_.run(dt, theta, theta_c, false, x.omega(1), true);
  u(quadrotor::TAUZ) = yaw_rate_.run(dt, x.omega(2), r_c, false, true);

  // Log all data
  //log(t, u);
  return u;
}

Controller::PID::PID() :
  kp_(0.0f),
  ki_(0.0f),
  kd_(0.0f),
  max_(1.0f),
  integrator_(0.0f),
  differentiator_(0.0f),
  prev_x_(0.0f),
  tau_(0.05)
{}

void Controller::PID::init(float kp, float ki, float kd, float max, float min, float tau)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  max_ = max;
  tau_ = tau;
}

float Controller::PID::run(float dt, float x, float x_c, bool update_integrator, bool update)
{
  float xdot;
  if (dt > 0.0001f)
  {
    if(update)
    {
      // calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
      // The dirty derivative is a sort of low-pass filtered version of the derivative.
      //// (Include reference to Dr. Beard's notes here)
      differentiator_ = (2.0f * tau_ - dt) / (2.0f * tau_ + dt) * differentiator_
      + 2.0f / (2.0f * tau_ + dt) * (x - prev_x_);
      xdot = differentiator_;
    }
    else
    {
      xdot = (2.0f * tau_ - dt) / (2.0f * tau_ + dt) * differentiator_
      + 2.0f / (2.0f * tau_ + dt) * (x - prev_x_);
    }


  }
  else
  {
    xdot = 0.0f;
  }
  if(update)
    prev_x_ = x;

  return run(dt, x, x_c, update_integrator, xdot, update);
}

float Controller::PID::run(float dt, float x, float x_c, bool update_integrator, float xdot, bool update)
{
  // Calculate Error
  float error = x_c - x;

  // Initialize Terms
  float p_term = error * kp_;
  float i_term = 0.0f;
  float d_term = 0.0f;

  if(error > 0)
     double t  = 0.0;

  // If there is a derivative term
  if (kd_ > 0.0f)
  {
    d_term = kd_ * xdot;
  }

  //If there is an integrator term and we are updating integrators
  if ((ki_ > 0.0f) && update_integrator)
  {
    if(update)
    {
      // integrate
      integrator_ += error * dt;
      // calculate I term
      i_term = ki_ * integrator_;
    }
    else
    {
      i_term = ki_ * (integrator_ + error * dt);
    }

  }

  // sum three terms
  float u = p_term - d_term + i_term;

  // Integrator anti-windup
  float u_sat = (u > max_) ? max_ : (u < -1.0 * max_) ? -1.0 * max_ : u;
  if (u != u_sat && fabs(i_term) > fabs(u - p_term + d_term) && ki_ > 0.0f)
  {
    if(update)
      integrator_ = (u_sat - p_term + d_term)/ki_;
  }

  // Set output
  return u_sat;
}

void Controller::log(const double &t, const uVector& u)
{
  // Write data to binary files and plot in another program
  // vehicle::xVector commanded_state = xc_.toEigen();
  // command_state_log_.write((char*)&t, sizeof(double));
  // command_state_log_.write((char*)commanded_state.data(), commanded_state.rows() * sizeof(double));
  // command_log_.write((char*)&t, sizeof(double));
  // command_log_.write((char*)u.data(), u.rows() * sizeof(double));
  // euler_command_log_.write((char*)&t, sizeof(double));
  // euler_command_log_.write((char*)xc_.q.euler().data(), 3 * sizeof(double));
  // target_log_.write((char*)&t, sizeof(double));
  // target_log_.write((char*)z_.data(), z_.rows() * sizeof(double));
  // target_log_.write((char*)vz_.data(), vz_.rows() * sizeof(double));
}



// Matrix<double,3,1> Controller::calculate_VO(vehicle::Stated x,
//   std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_positions,
//   std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_velocities,
//   Matrix<double,3,1> desVelocity)
// {
//   geometry_msgs::Point outPoint;
//   geometry_msgs::Vector3 outVector;
//   for (int i = 0; i < other_vehicle_positions.size(); ++i)
//   {
//     outPoint.x = other_vehicle_positions[i][0];
//     outPoint.y = other_vehicle_positions[i][1];
//     outPoint.z = other_vehicle_positions[i][2];
//     srv_vo_.request.otherPositions.push_back(outPoint);
//
//     outVector.x = other_vehicle_velocities[i][0];
//     outVector.y = other_vehicle_velocities[i][1];
//     outVector.z = other_vehicle_velocities[i][2];
//     srv_vo_.request.otherVelocities.push_back(outVector);
//
//   }
//
//   outPoint.x = x.p[0];
//   outPoint.y = x.p[1];
//   outPoint.z = x.p[2];
//   srv_vo_.request.currentPosition = outPoint;
//
//   outVector.x = x.v[0];
//   outVector.y = x.v[1];
//   outVector.y = x.v[2];
//   srv_vo_.request.currentVelocity = outVector;
//
//   outVector.x = desVelocity[0];
//   outVector.y = desVelocity[1];
//   outVector.y = desVelocity[2];
//   srv_vo_.request.desVelocity = outVector;
//
//   vo_client_->call(srv_vo_);
//   Matrix<double,3,1> output;
//   output << srv_vo_.response.desVelocity[0],
//             srv_vo_.response.desVelocity[1],
//             desVelocity[2];
//   return output;
// }

} // namespace quadrotor
