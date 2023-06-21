#include "collision_vo/collision_vo.h"

CollisionVO::CollisionVO() {};

CollisionVO::~CollisionVO() {}

Vec2d CollisionVO::get_best_vel(const Vec2d& av1Pos,
  const Vec2d& av1Vel,
  const Vec2d& av1VelDes,
  const Eigen::VectorXd& state,
  const double& dt,
  const std::vector<Vec2d>& inRangePos,
  const std::vector<Vec2d>& inRangeVel,
  const std::vector<Vec2d>& posUncertainty,
  const std::vector<Vec2d>& velUncertainty)
{
  // set_admissible_velocities(fmax,numPointsAdmissible,state,dt,admissibleVelocities);

  get_admissible_velocities(state, numPointsAdmissible);

  std::vector<Vec2d> convexHull;
  int n = admissibleVelocities.size();
  convex_hull(admissibleVelocities, n, convexHull);


  std::vector<double> x_curve;
  std::vector<double> y_curve;
  std::vector<double> z_curve;

  // Implemnet Buffer Zone Idea Here:
  Vec2d av1BuffVel;
  if(bufferOn)
    av1BuffVel = CalculateBuffer(av1Pos, inRangePos, collisionRadius, bufferPower);
  else
    av1BuffVel.setZero();

  // Vec2d tempVelDes =  av1VelDes + av1BuffVel;

  Vec2d av1Xo = av1Pos;
  Vec2d av1Vo = av1Vel;
  Vec2d av1PreferedVoD = (2*av1VelDes + av1BuffVel)/2.0;
  // if(av1BuffVel.norm() > 1.0)
  // {
  //   av1PreferedVoD = av1BuffVel;
  //   // std::cout << "here: " <<  av1Pos << std::endl;
  //   // std::cout << av1PreferedVoD << std::endl;
  // }

  Vec2d vectorFieldAdd(0,0);
  int numTooClose(0);

  Vec2d winVoD = av1Vel;
  double winPenalty = std::numeric_limits<double>::infinity();

  for(int i(0);i<admissibleVelocities.size();i++)
  {
    Vec2d point = admissibleVelocities[i];

    double penalty = (av1PreferedVoD-point).norm();
//    double penalty = 0.0;
    for(int j(0);j<inRangePos.size();j++)
    {
      Vec2d av2Xo = inRangePos[j];
      Vec2d av2Vo = inRangeVel[j];

      double collisionDistance{0};

      Intersection intercection = get_distance(av1Xo, av1Vo, av2Xo, av2Vo, point, collisionDistance);

      if(intercection == INTERSECT)
      {
        double collisionTime = get_collision_time(collisionDistance, av1Xo, av1Vo, av2Vo, point);
        penalty += timeCollisionPenality / collisionTime;
      }

      if(intercection != NEGATIVE && uncertaintyOn)
      {
        double omegaPos = 0;
        double omegaVel = 0;

        if(true)
        {
          Vec2d A = av1Xo - av2Xo;

          Vec2d VoD = point;
          Vec2d VoDD = 2*VoD-av1Vo-av2Vo;
          Vec2d P = A + VoDD;


          double a = posUncertainty[j][0]+2.0*collisionRadius;
          double b = posUncertainty[j][1]+2.0*collisionRadius;

          Vec2d uncertainty{a,b};

          double distance;

          line_ellipse_intersect(A,P,uncertainty,distance,omegaPos);

          // penalty += 1.0 / (1.0+exp(1000.0*(omegaPos-uncertainVelPenality)));
         // penalty += 1.0/omega * uncertainPosPenality;


        }

        if(true)
        {
          Vec2d A = av1Xo - (av1Xo+av1Vo-av2Vo);

          Vec2d P = 2*point-av1Vo-av2Vo;

          Vec2d uncertainty{velUncertainty[j][0], velUncertainty[j][1]};

          double distance;

          line_ellipse_intersect(A,P,uncertainty,distance,omegaVel);

          // penalty += 1.0/omega * uncertainVelPenality;

          // penalty += uncertainVelPenality * 1.0 / (1.0+exp(100.0*(s-1)));

          penalty += 1.0 / (1.0+exp(1000.0*((omegaPos+omegaVel)-(uncertainPosPenality+uncertainVelPenality))));
        }
      }



    }
    z_curve.push_back(penalty);
    if(penalty < winPenalty)
    {
      winPenalty = penalty;
      winVoD = point + av1Xo;
    }

  }
  for(auto point : admissibleVelocities)
  {
    x_curve.push_back(point[0]);
    y_curve.push_back(point[1]);


  }




  Eigen::VectorXd x_c = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(x_curve.data(), x_curve.size());
  Eigen::VectorXd y_c = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(y_curve.data(), y_curve.size());
  Eigen::VectorXd z_c = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(z_curve.data(), z_curve.size());


  Vec2d av1VoD = winVoD-av1Xo;

  Eigen::VectorXd Coef(get_sum(order));
  curve_fit(order,x_c,y_c,z_c,Coef);
  Vec2d initSolution{av1VoD[0],av1VoD[1]};

  // minimize(order,Coef,convexHull,initSolution);

  av1VoD = initSolution;

//  std::cout << "points = np.array([\n";
//  for(int i(0); i < x_curve.size(); i++)
//  {
//    std::cout << "[" << x_curve[i] << "," << y_curve[i] << "," << z_curve[i] << "],\n";
//  }
//  std::cout << "])\n";

////  std::cout << "all_s = np.array([";
////  for(int i(0);i<all_s.size();i++)
////  {
////      std::cout << all_s[i] << ",\n";
////  }
////  std::cout << "])\n";

//  std::cout << "min = np.array([" << initSolution[0] << ", " << initSolution[1] << "])\n";

//  std::cout << "posA = np.array([" << av1Xo[0] << ", " << av1Xo[1] << "])\n";
//  std::cout << "velA = np.array([" << av1Vo[0] << ", " << av1Vo[1] << "])\n";
//  if(inRangePos.size()>0)
//  {
//      std::cout << "posB = np.array([" << inRangePos[0][0] << ", " << inRangePos[0][1] << "])\n";
//      std::cout << "velB = np.array([" << inRangeVel[0][0] << ", " << inRangeVel[0][1] << "])\n";

//      std::cout << "a_pos = " << posUncertainty[0][0] << "+2.0*radius\n";
//      std::cout << "b_pos = " << posUncertainty[0][1] << "+2.0*radius\n";
//      std::cout << "a_vel = " << velUncertainty[0][0] << "\n";
//      std::cout << "b_vel = " << velUncertainty[0][1] << "\n";
//  }

//  std::cout << "\n";


  return av1VoD;

}

Intersection CollisionVO::get_distance(const Vec2d& av1Xo,
  const Vec2d& av1Vo,
  const Vec2d& av2Xo,
  const Vec2d& av2Vo,
  const Vec2d& point,
  double& distance)
{
  Vec2d A = av1Xo - av2Xo;

  Vec2d VoD = point;
  Vec2d VoDD = 2*VoD-av1Vo-av2Vo;
  Vec2d P = A + VoDD;
  Vec2d uncertainty{2*collisionRadius,2*collisionRadius};
  double omega{0.0};
  Intersection intersect = line_ellipse_intersect(A,P,uncertainty,distance,omega);
  return intersect;
}

Intersection CollisionVO::line_ellipse_intersect(const Vec2d& A,
  const Vec2d& P,
  const Vec2d& uncertainty,
  double& distance,
  double& omega)
{

  double q1 = 0, q2 = 0, q3 = 0;
  std::vector<double> q3_a;

  for(int i{0};i<2;i++)
  {
    q1 += ( A[i]*A[i] - 2.0*A[i]*P[i] + P[i]*P[i] ) / pow(uncertainty[i],2.0);
    q2 += ( 2.0*A[i]*P[i] - 2.0*A[i]*A[i] ) / pow(uncertainty[i],2.0);
    q3_a.push_back( ( A[i]*A[i] ) / pow(uncertainty[i],2.0) );
    q3 += q3_a[i];
  }
  q3 -= 1.0;

  omega = sqrt( (-q2*q2 + 4*q1*(q3_a[0]+q3_a[1]+q3_a[2]) )/(4*q1));
  double r1, r2, i1, i2;

  if(!quadratic_formula(q1,q2,q3,r1,r2,i1,i2))
    return NO_INTERSECT;

  distance = r1 < r2 ? r1: r2;

  distance *= (P-A).norm();



  if(distance < 0)
    return NEGATIVE;

  return INTERSECT;

}



double CollisionVO::get_collision_time(const double& distance,
  const Vec2d& av1Xo,
  const Vec2d& av1Vo,
  const Vec2d& av2Vo,
  const Vec2d& point)
{
  Vec2d VoD = point;
  Vec2d VoDD = 2.0*VoD-av1Vo-av2Vo;

  double time  =  distance / VoDD.norm();

  return time;
}


void CollisionVO::get_admissible_velocities(const Eigen::VectorXd& state, int N)
{
  admissibleVelocities = std::vector<Vec2d>();
  //std::cout << "Here: " <<  Ts << std::endl;
  double distanceAway = Ts * maxAccel; //Ts is the time step.
  // std::cout << N << std::endl;
  std::vector<double> radii = linspace(0,distanceAway,N);
  std::vector<double> numPoints = linspace(1,N*N,N);

  for(int i(0);i<radii.size();i++)
  {
    std::vector<double> theta = linspace(0,2*PI,numPoints[i]);
    for(int j(0);j<theta.size();j++)
    {
      double x = cos(theta[j]) * radii[i];
      double y = sin(theta[j]) * radii[i];

      Vec2d newVel = Vec2d(x,y)+state.segment<2>(3);

      // if(newVel.norm()<3.0)
      //   continue;

      admissibleVelocities.push_back(newVel);
    }
  }

}
