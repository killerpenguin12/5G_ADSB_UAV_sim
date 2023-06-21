#include "common_cpp/common.h"
// #include "common_cpp/quaternion.h"
// #include "common_cpp/transform.h"
#include <random>
#include <chrono>

using namespace std;
using namespace Eigen;



int main()
{
  // Update this to use Google Test sometime
//  // Error tolerance and number of common::TESTs
//  double tol = 1e-6;
//  int N = 1000;
//  srand((unsigned)time(NULL));

//  // Initialize Gaussian random number generation
//  unsigned seed = chrono::system_clock::now().time_since_epoch().count();
//  default_random_engine rng(seed);
//  srand(seed);
//  normal_distribution<double> dist(0.0,0.1);

//  for (int i = 0; i < N; ++i)
//  {
//    Vector3d p1; p1.setRandom();
//    common::Quaterniond q1(dist,rng);
//    Vector3d delta_p, delta_q;
//    delta_p.setRandom();
//    delta_q.setRandom();
//    delta_p *= 0.5;
//    delta_q *= 0.5;
//    if (delta_q.norm() > M_PI) continue;

//    // Check boxplus and boxminus operations
//    common::Quaterniond q2 = q1 + delta_q;
//    Vector3d delta_q_new = q2 - q1;
//    if (!common::TEST("Quaternion boxplus and boxminus operations.",tol,delta_q,delta_q_new)) break;

//    // Check rotation matrix exponential
//    Matrix3d R1 = q1.R();
//    Matrix3d R2 = q2.R();
//    Vector3d deltaR = -R1.transpose()*delta_q;
//    Matrix3d R2_new = R1*common::expR(common::skew(deltaR));
//    if (!common::TEST("Rotation matrix exponential.",tol,R2,R2_new)) break;

//    // Check rotation matrix logarithm
//    Matrix3d dR = R1*R2.transpose();
//    delta_q_new = common::vex(common::logR(dR));
//    if (!common::TEST("Rotation matrix logarithm.",tol,delta_q,delta_q_new)) break;

//    // Check rotation matrix transpose exponential
//    R1 = q1.R().transpose();
//    R2 = q2.R().transpose();
//    R2_new = R1*common::expR(common::skew(delta_q));
//    if (!common::TEST("Rotation matrix transpose exponential.",tol,R2,R2_new)) break;

//    // Check rotation matrix transpose logarithm
//    dR = R1.transpose()*R2;
//    delta_q_new = common::vex(common::logR(dR));
//    if (!common::TEST("Rotation matrix transpose logarithm.",tol,delta_q,delta_q_new)) break;

//    // Check transform boxplus and boxminus operators
//    common::Transformd t1(p1, q1);
//    Matrix<double,6,1> delta_t;
//    delta_t.segment<3>(0) = delta_p;
//    delta_t.segment<3>(3) = delta_q;
//    common::Transformd t2 = t1 + delta_t;
//    Matrix<double,6,1> delta_t2 = t2 - t1;
//    if (!common::TEST("Transform boxplus and boxminus operators.",tol,delta_t,delta_t2)) break;

//    // Check unit vector boxplus and boxminus
//    {
//    Vector3d u1;
//    u1.setRandom();
//    common::Quaterniond q1(u1);
//    Vector2d delta;
//    delta.setRandom();
//    common::Quaterniond q2 = common::Quaterniond::exp(q1.proj() * delta) * q1;
//    Vector2d delta2 = common::Quaterniond::log_uvec(q2, q1);
//    if (!common::TEST("Unit vector Quaternion boxplus and boxminus operators.",tol,delta,delta2)) break;
//    }
//  }
}
