#include "gen_kalman_filter.h"
#include <iostream>

GenKalmanFilter::GenKalmanFilter() {}
GenKalmanFilter::~GenKalmanFilter() {}

void GenKalmanFilter::init(double sigmaQ_vel, double alphaQ_vel,
  double sigmaQ_jrk, double alphaQ_jrk,
  double sigmaR_pos, double sigmaR_vel,
  double sigmaR_range, double sigmaR_zenith,
    enum MotionModelType mmtype, double dt,
    uint32_t owner_id, uint32_t id, VectorXd x)
{

  sigmaQ_vel_ = sigmaQ_vel;
  alphaQ_vel_ = alphaQ_vel;
  sigmaQ_jrk_ = sigmaQ_jrk;
  alphaQ_jrk_ = alphaQ_jrk;

  sigmaR_pos_ = sigmaR_pos;
  sigmaR_vel_ = sigmaR_vel;
  sigmaR_range_ = sigmaR_range;
  sigmaR_zenith_ = sigmaR_zenith;

  id_ = id;

  xhat_ = x;

  meas_dim_ = 2; //Hardcoded for now

  // Set the state dimension based on the motion model type
  if (mmtype == CONSTANT_VEL)
      n = meas_dim_*2; // xhat = [pos vel]^T

  else if (mmtype == CONSTANT_ACCEL)
      n = meas_dim_*3; // xhat = [pos vel acc]^T

  else if (mmtype == CONSTANT_JERK)
      n = meas_dim_*4; // xhat = [pos vel acc jerk]^T

  // The timestep is used to generate
  // state transition and process noise covariance matrices
  dt_ = dt;

  mmtype_ = mmtype;

  A_ = build_A();
  Q_ = build_Q();

  C_pos_ = build_C(2, n);
  C_vel_ = build_C(4, n);
  R_pos_ = build_R(2);
  //std::cout << "Does this get called? " << std::endl;
  R_vel_ = build_R(4);
  //std::cout << "Nope " << std::endl;
  P_ = MatrixXd::Identity(n,n)*0.1;
  P_(0,0) = 1;
  P_(1,1) = 1;
  // P_(2,2) = 1;
  // P_(3,3) = 1;
  

  // Initialize loggers and log initial data
  //std::stringstream ss_s;
  //ss_s << "/tmp/quad" << owner_id << "_" << id_ << "_kalman_state.log";
  //estimate_log_.open(ss_s.str());
  // log(0);

}

void GenKalmanFilter::predict(const double &t)
{
  // KF predict step
  xhat_ = A_*xhat_;
  P_ = A_*P_*A_.transpose() + Q_;
  log(t);
}

void GenKalmanFilter::update(const VectorXd& measurement, bool hasVel)
{
  
  Eigen::MatrixXd C;
  Eigen::MatrixXd R;
  //std::cout << "hasVel: " << hasVel << std::endl;
  if(hasVel)
  {
    //std::cout << "R: " << R << std::endl;
    //std::cout << "R_vel_: " << R_vel_ << std::endl;
    R = R_vel_; //R_vel_ is the problem and causes the seg fault
    C = C_vel_;

  }
  else
  {
    R = R_pos_;
    C = C_pos_;
  }

  // Calculate Kalman gain
  Eigen::MatrixXd K = P_*C.transpose()*(C*P_*C.transpose() + R).inverse();
  xhat_ = xhat_ + K*(measurement-C*xhat_);
  Eigen::MatrixXd eye_n = Eigen::MatrixXd::Identity(n, n);
  P_ = (eye_n - K*C)*P_;

}

void GenKalmanFilter::update_radar(const Vector2d& measurement, const Vector2d& radarPos)
{
  Eigen::MatrixXd C;
  Eigen::MatrixXd R;
  Eigen::Vector2d radarMeasurement = measurement - radarPos;

  double range = radarMeasurement.norm();
  double zenith = atan2(radarMeasurement(0), radarMeasurement(1));
  R = build_polar_R(range, zenith);
  C = C_pos_;


  // Calculate Kalman gain
  Eigen::MatrixXd K = P_*C.transpose()*(C*P_*C.transpose() + R).inverse();
  xhat_ = xhat_ + K*(measurement-C*xhat_);
  Eigen::MatrixXd eye_n = Eigen::MatrixXd::Identity(n, n);
  P_ = (eye_n - K*C)*P_;

}

Eigen::MatrixXd GenKalmanFilter::build_A()
{
  /**
   * For n=4 (2D, constant velocity) this will build
   * the state transition matrix that looks like:
   *
   *            [ 1  0  dt 0  ]
   *        A = [ 0  1  0  dt ]
   *            [ 0  0  1  0  ]
   *            [ 0  0  0  1  ]
   *
   * (See Ingersoll, Thesis, eq. 6.1, 6.2, 6.3 for NCV, NVA, NVJ models)
   **/

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n,n);
  for (uint32_t i=0; i<n; i++) {
      // Make the diagonals 1's
      A(i, i) = 1;

      // make the uppper-triangular off-diagonals equal
      // to ((dt)^j)/j by iterating down the row.
      for (uint32_t j=1; j<n/2; j++) {
          // this check ensures that we don't fall off the end of a row.
          if (i+(j*2) < n)
              A(i, i+(j*2)) = pow(dt_,j)/j;
      }
  }

  return A;
}

Eigen::MatrixXd GenKalmanFilter::build_Q()
{
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n,n);

  // TODO: the above Q uses the correct n, adjust the logic below to account
  // for the environment dimensionality. block matrices of pow(dt_,M)/N
  // which get concatenated, resulting in a guaranteed nxn. DIM

  if (mmtype_ == CONSTANT_VEL) {


      Q <<  pow(dt_,4)/4 ,      0       , pow(dt_,3)/2 ,      0       ,
                 0       , pow(dt_,4)/4 ,      0       , pow(dt_,3)/2 ,
            pow(dt_,3)/2 ,      0       , pow(dt_,2)/1 ,      0       ,
                 0       , pow(dt_,3)/2 ,      0       , pow(dt_,2)/1 ;

      Q = 2 * alphaQ_vel_ * pow(sigmaQ_vel_,2) * Q;

      // sigmaQ^2 is the variance of the target velocity
      // alphaQ   is the inverse time constant of the change in velocity

      // It isn't clear if alphaQ_vel_ works this way. By setting it to 0.5,
      // the formula simplifies to match (Niedfeldt, Dissertation, eq 2.16)


  } else if (mmtype_ == CONSTANT_ACCEL) {
      // TODO
      throw std::logic_error("Constant Acceleration model not implemented");

  } else if (mmtype_ == CONSTANT_JERK) {


      Q <<  pow(dt_,7)/252 ,       0        , pow(dt_,6)/72  ,       0        , pow(dt_,5)/30  ,       0        , pow(dt_,4)/24  ,       0        ,
                  0        , pow(dt_,7)/252 ,       0        , pow(dt_,6)/72  ,       0        , pow(dt_,5)/30  ,       0        , pow(dt_,4)/24  ,
            pow(dt_,6)/72  ,       0        , pow(dt_,5)/20  ,       0        , pow(dt_,4)/8   ,       0        , pow(dt_,3)/6   ,       0        ,
                  0        , pow(dt_,6)/72  ,       0        , pow(dt_,5)/20  ,       0        , pow(dt_,4)/8   ,       0        , pow(dt_,3)/6   ,
            pow(dt_,5)/30  ,       0        , pow(dt_,4)/8   ,       0        , pow(dt_,3)/3   ,       0        , pow(dt_,2)/2   ,       0        ,
                  0        , pow(dt_,5)/30  ,       0        , pow(dt_,4)/8   ,       0        , pow(dt_,3)/3   ,       0        , pow(dt_,2)/2   ,
            pow(dt_,4)/24  ,       0        , pow(dt_,3)/6   ,       0        , pow(dt_,2)/2   ,       0        , pow(dt_,1)/1   ,       0        ,
                  0        , pow(dt_,4)/24  ,       0        , pow(dt_,3)/6   ,       0        , pow(dt_,2)/2   ,       0        , pow(dt_,1)/1   ;

      Q = 2 * alphaQ_jrk_ * pow(sigmaQ_jrk_,2) * Q;

      // sigmaQ^2 is the variance of the target jerk
      // alphaQ   is the inverse time constant of the change in jerk

      // Ingersoll, Thesis, 6.1
      // Mehrota, K. 1997, Jerk Model for Tracking Highly Maneuvering Targets
      // Q comes from Mehrota eq. 21 (fixed from the original code). SigmaQ
      // and alpha are tuning parameters - small alpha for targets with
      // sustained jerk and high for targets with rapidly fluctuating jerk.

  }

  return Q;
}

Eigen::MatrixXd GenKalmanFilter::build_C(uint32_t m_request, uint32_t n_request)
{
    return Eigen::MatrixXd::Identity(m_request, n_request);
}


Eigen::MatrixXd GenKalmanFilter::build_R(uint32_t m_request) {
    /**
     * Measurement Noise Covariance
     * See (Niedfeldt, Dissertation, last sentence on p. 31)
     *
     **/
    //std::cout << "m_request: " << m_request << std::endl;
    meas_dim_ = 2;
    int32_t m_vel = meas_dim_*2;
    // Create an (m_vel x m_vel) size identity matrix, but make the last two
    // ones in the diagonal zeos (the ones that correspond to a vel meas)
    Eigen::MatrixXd Im_pos = Eigen::MatrixXd::Identity(m_vel, m_vel);
    Im_pos(2, 2) = 0;
    Im_pos(3, 3) = 0;
    //std::cout << "Im_pos: " << Im_pos << std::endl;
    // TODO: this is hardcoded for meas_dim_=2 make these zero commands
    // based on meas_dim_ (below as well); DIM

    // Create an (m_vel x m_vel) size identity matrix, but make the first two
    // ones in the diagonal zeros (the ones that correspond to a pos meas)
    Eigen::MatrixXd Im_vel = Eigen::MatrixXd::Identity(m_vel, m_vel);
    Im_vel(0, 0) = 0;
    Im_vel(1, 1) = 0;
    //std::cout << "Im_vel: " << Im_vel << std::endl;

    // This lets the covariance for pos and vel measurements be different
    Eigen::MatrixXd R_whole = Im_pos*pow(sigmaR_pos_,2) + Im_vel*pow(sigmaR_vel_,2);
    //std::cout << "R_whole: " << R_whole << std::endl;
    // Save a submat acording to the source (based on size m)
    return R_whole.block(0, 0, m_request, m_request);

}


Eigen::MatrixXd GenKalmanFilter::build_polar_R(const double& range, const double& zenith)
{
  Eigen::Matrix2d R_radar = Eigen::Vector2d(sigmaR_range_*sigmaR_range_, sigmaR_zenith_*sigmaR_zenith_).asDiagonal();
  Eigen::Matrix2d B;
  B << sin(zenith), range*cos(zenith), cos(zenith), -range*sin(zenith);
  return B*R_radar*B.transpose();
}

void GenKalmanFilter::log(const double &t)
{
  // Write data to binary files and plot in another program
//  estimate_log_.log(t);
//  estimate_log_.logMatrix(xhat_);
//  estimate_log_.logMatrix(P_);
}
