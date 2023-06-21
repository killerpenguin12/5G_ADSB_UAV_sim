#pragma once

#include <eigen3/Eigen/Eigen>
#include "geometry/quat.h"

using namespace Eigen;


namespace vehicle
{

// State Indices
enum
{
  P = 0, // Position
  V = 3, // Velocity
  LA = 6, // Linear acceleration
  Q = 9, // Attitude
  W = 13, // Angular rate
  AA = 16, // Angular acceleration
  NUM_STATES = 19
};

// State Derivative Indices
enum
{
  DP = 0,
  DV = 3,
  DQ = 6,
  DW = 9,
  NUM_DOF = 12
};

typedef Matrix<double, NUM_STATES, 1> xVector;
typedef Matrix<double, NUM_STATES, NUM_STATES> xMatrix;
typedef Matrix<double, NUM_DOF, 1> dxVector;
typedef Matrix<double, NUM_DOF, NUM_DOF> dxMatrix;

template<typename T>
struct State
{

  Matrix<T,3,1> p;
  Matrix<T,3,1> v;
  Matrix<T,3,1> lin_accel;
  quat::Quat<T> q;
  Matrix<T,3,1> omega;
  Matrix<T,3,1> ang_accel;
  T drag; // quadrotor horizontal drag

  State()
  {
    p.setZero();
    v.setZero();
    lin_accel.setZero();
    omega.setZero();
    ang_accel.setZero();
  }

  State(const Matrix<T, NUM_STATES, 1>  &x0)
  {
    p = x0.template segment<3>(P);
    v = x0.template segment<3>(V);
    lin_accel = x0.template segment<3>(LA);
    q = quat::Quat<T>(x0.template segment<4>(Q));
    omega = x0.template segment<3>(W);
    ang_accel = x0.template segment<3>(AA);
  }

  State operator+(const Matrix<T, NUM_DOF, 1> &delta) const
  {
    State x;
    x.p = p + delta.template segment<3>(DP);
    x.v = v + delta.template segment<3>(DV);
    x.q = q + delta.template segment<3>(DQ);
    x.omega = omega + delta.template segment<3>(DW);
    x.drag = drag;
    return x;
  }

  void operator+=(const Matrix<T, NUM_DOF, 1> &delta)
  {
    *this = *this + delta;
  }

  dxVector operator-(const State<T> &x) const
  {
    dxVector dx;
    dx.template segment<3>(DP) = p - x.p;
    dx.template segment<3>(DV) = v - x.v;
    dx.template segment<3>(DQ) = q - x.q;
    dx.template segment<3>(DW) = omega - x.omega;
    return dx;
  }

  void setZero()
  {
    p.setZero();
    v.setZero();
    lin_accel.setZero();
    q = quat::Quat<T>();
    omega.setZero();
    ang_accel.setZero();
  }
  void setP(const Eigen::Vector3d& vw)
  {
    p = vw;
  }
  Matrix<T, NUM_STATES, 1> toEigen() const
  {
    Matrix<T, NUM_STATES, 1> x;
    x << p, v, lin_accel, q.elements(), omega, ang_accel;
    return x;
  }

  Matrix<T, NUM_DOF, 1> minimal() const
  {
    Matrix<T, NUM_DOF, 1> x;
    x << p, v, lin_accel, quat::Quat<T>::log(q), omega, ang_accel;
    return x;
  }

};
typedef State<double> Stated;


// 4th order integration for truth of each vehicle
template<int U> // command vector size
void rk4(std::function<void(const Stated&, const Matrix<double,U,1>&, const Vector3d&, dxVector&)> f,
                            const double& dt, const Stated& x, const Matrix<double,U,1>& u,
                            const Vector3d& vw, vehicle::dxVector& dx)
{
  dxVector k1, k2, k3, k4;
  f(x, u, vw, k1);
  f(x + k1 * dt / 2, u, vw, k2);
  f(x + k2 * dt / 2, u, vw, k3);
  f(x + k3 * dt, u, vw, k4);
  dx = (k1 + 2 * k2 + 2 * k3 + k4) * dt / 6.0;
}

} // namespace vehicle


// Command vectors to be shared with controllers
namespace quadrotor
{

enum
{
  THRUST,
  TAUX,
  TAUY,
  TAUZ,
  COMMAND_SIZE
};
typedef Matrix<double, COMMAND_SIZE, 1> uVector;

} // namespace quadrotor


namespace fixedwing
{

enum
{
  AIL, // aileron
  ELE, // elevator
  THR, // throttle
  RUD, // rudder
  COMMAND_SIZE
};
typedef Matrix<double, COMMAND_SIZE, 1> uVector;
typedef Matrix<double, COMMAND_SIZE, COMMAND_SIZE> uMatrix;

} // namespace fixedwing
