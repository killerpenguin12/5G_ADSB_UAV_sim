#include "collision_vo/admissible_velocities.h"

Eigen::Matrix3d RtoQ(const Eigen::Vector4d& q)
{
  double w = q[0];
  double x = q[1];
  double y = q[2];
  double z = q[3];
  double wx = w*x;
  double wy = w*y;
  double wz = w*z;
  double xx = x*x;
  double xy = x*y;
  double xz = x*z;
  double yy = y*y;
  double yz = y*z;
  double zz = z*z;
  Eigen::Matrix3d out;
  out << 1. - 2.*yy - 2.*zz, 2.*xy + 2.*wz,      2.*xz - 2.*wy,
         2.*xy - 2.*wz,      1. - 2.*xx - 2.*zz, 2.*yz + 2.*wx,
         2.*xz + 2.*wy,      2.*yz - 2.*wx,      1. - 2.*xx - 2.*yy;
  return out;
}

void exp(const Eigen::VectorXd& v,
  Eigen::VectorXd& q)
{
  double norm_v = v.norm();
  if(norm_v > 1e-4)
  {
    double v_scale = sin(norm_v/2.0)/norm_v;
    q  = Eigen::Vector4d(cos(norm_v/2.0), v_scale*v[0], v_scale*v[1], v_scale*v[2]);
  }
  else
  {
    q  = Eigen::Vector4d(1.0, v[0]/2.0, v[1]/2.0, v[2]/2.0).normalized();
  }
}

Eigen::VectorXd boxplus(const Eigen::VectorXd& state,
  const Eigen::VectorXd& delta)
{
  Eigen::VectorXd q1 = state.segment<4>(6);
  Eigen::VectorXd q2;

  exp(delta.segment<3>(6), q2);

  double q1w = q1[0];
  double q1x = q1[1];
  double q1y = q1[2];
  double q1z = q1[3];
  double q2w = q2[0];
  double q2x = q2[1];
  double q2y = q2[2];
  double q2z = q2[3];

  Eigen::Vector4d dq;
  dq[0] = q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z;
  dq[1] = q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y;
  dq[2] = q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x;
  dq[3] = q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w;

  Eigen::VectorXd returnState(13);
  returnState.setZero();

  returnState.segment<6>(0) = state.segment<6>(0) + delta.segment<6>(0);
  returnState.segment<4>(6) = dq;
  returnState.segment<3>(10) = state.segment<3>(9) + delta.segment<3>(9);

  return returnState;
}

Eigen::VectorXd derivatives(const Eigen::VectorXd& state,
  const Eigen::Vector4d& u)
{
  Eigen::Vector3d pos = state.segment<3>(0);
  Eigen::Vector3d vel = state.segment<3>(3);
  Eigen::Vector4d q = state.segment<4>(6);
  Eigen::Matrix3d R = RtoQ(q);
  Eigen::Vector3d omega = state.segment<3>(10);

  double F = u[0];
  Eigen::Vector3d TAU = u.segment<3>(1);

  Eigen::VectorXd returnDx(12);

  returnDx.segment<3>(0) = R.inverse() * vel;

  returnDx.segment<3>(3) = -e3 * F * max_thrust / mass -
      linear_drag_matrix * vel +
      g * R * e3 -
      omega.cross(vel);

  returnDx.segment<3>(6) = omega;

  Eigen::Vector3d incross = (inertia_matrix * omega);

  returnDx.segment<3>(9) = inertia_inv * (TAU - omega.cross(incross) -
    angular_drag_matrix * omega);

  return returnDx;
}


void propagateDynamics(const Eigen::MatrixXd& state,
  const Eigen::VectorXd& u,
  const double& dt,
  Eigen::VectorXd& returnState)
{
  // Integrate ODE using Runge-Kutta RK4 algorithm
  Eigen::VectorXd k1 = derivatives(state, u);
  Eigen::VectorXd k2 = derivatives(boxplus(state, dt/2.0*k1), u);
  Eigen::VectorXd k3 = derivatives(boxplus(state, dt/2.0*k2), u);
  Eigen::VectorXd k4 = derivatives(boxplus(state, dt*k3), u);

  returnState = boxplus(state, dt/6.0 * (k1 + 2.0*k2 + 2.0*k3 + k4));
}


void set_admissible_velocities(const int numPoints,
  const Eigen::VectorXd state,
  const double& dt,
  std::vector<Eigen::Vector2d>& admissibleVelocities)
{

  std::vector<double> tau_phi = linspace(-200, 200, numPoints);
  std::vector<double> tau_theta = linspace(-50, 50, numPoints);
  std::vector<double> tau_psi = linspace(-200, 200, numPoints);
  std::vector<double> F_all = linspace(0, 1, numPoints);


  Eigen::Vector2d vel{state[3],state[4]};
  admissibleVelocities.push_back(vel);

  for(auto F : F_all)
  {
    for(auto t_phi : tau_phi)
    {
      for(auto t_theta : tau_theta)
      {
        for(auto t_psi : tau_psi)
        {
          Eigen::Vector4d u(F, t_phi, t_theta, t_psi);
          Eigen::VectorXd returnState;
          propagateDynamics(state, u, dt, returnState);

          vel = Eigen::Vector2d{returnState[3],returnState[4]};
          admissibleVelocities.push_back(vel);
        }
      }
    }
  }

}

void set_up_A(int order,
  const Eigen::VectorXd& x,
  const Eigen::VectorXd& y,
  Eigen::MatrixXd& A)
{
  int column(0);
  for(int i(0); i<=order; i++)
  {
    for(int j(0); j<=i; j++)
    {
      A.col(column) = x.array().pow(i-j).cwiseProduct(y.array().pow(j));
      column++;
    }
  }
}

int get_sum(int order)
{
  int sum{0};
  for (int i(0); i <= order+1; i++)
  {
     sum += i;
  }
  return sum;
}

void curve_fit(int order,
  const Eigen::VectorXd& x,
  const Eigen::VectorXd& y,
  const Eigen::VectorXd& z,
  Eigen::VectorXd& Coef)
{
  int sum = get_sum(order);
  Eigen::MatrixXd A(x.rows(),sum);
  A.setZero();

  set_up_A(order,x,y,A);
  Coef = A.colPivHouseholderQr().solve(z);
}

void evaluate_surface(int order,
  const Eigen::VectorXd& x,
  const Eigen::VectorXd& y,
  const Eigen::VectorXd& Coef,
  Eigen::VectorXd& z)
{
  int sum = get_sum(order);
  Eigen::MatrixXd A(x.rows(),sum);
  A.setZero();

  set_up_A(order,x,y,A);
  z = A*Coef;
}
