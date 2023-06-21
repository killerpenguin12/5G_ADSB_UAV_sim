#include "quad_cvo_manager.h"

namespace quadrotor
{

CVOManager::CVOManager() {}
CVOManager::~CVOManager()
{
  for(int i(0);i<kalFilters.size();i++)
  {
    delete kalFilters[i];
  }
  // delete env_;
}

void CVOManager::load(const std::string &filename, 
    environment::Environment* env,
    double id, double kalman_dt, double dt, double randRange,double maxAccel)
{
  env_ = env;
  id_ = id;
  kalman_dt_ = kalman_dt;
  dt_ = dt;

  double cRadius, cRange, cPenalty, posPenalty, velPenalty, solve_order;
  int numPointsAdmissible, bufferPower;
  bool bufferOn, uncertaintyOn;
  //cRadius = 15; //# Radius for collision
   // Range for collision dectection
  // maxAccel = 10000; // # Acceleration
  // cPenalty = 10; //# Time penalty
  // posPenalty = 2;
  // velPenalty = 2;
  // numPointsAdmissible = 5;
  // solve_order = 3;
  // bufferOn = false;
  // bufferPower = 50;
  // uncertaintyOn = true;
  common::get_yaml_node("collision_radius", filename, cRadius);
  if(randRange == 0.0)
  {
    //std::cout << " Here?" << std::endl;
    //cRange = 100;//std::cout << "Here is the third" << std::endl;0;
    common::get_yaml_node("collision_range", filename, cRange);
  }
  else
    cRange = randRange;
  //common::get_yaml_node("collision_acceleration", filename, maxAccel);
  common::get_yaml_node("time_collision_penality", filename, cPenalty);
  common::get_yaml_node("pos_uncertain_penalty", filename, posPenalty);
  common::get_yaml_node("vel_uncertain_penalty", filename, velPenalty);
  common::get_yaml_node("num_points_admissible", filename, numPointsAdmissible);
  common::get_yaml_node("solve_order", filename, solve_order);

  common::get_yaml_node("buffer_on", filename, bufferOn);
  common::get_yaml_node("buffer_power", filename, bufferPower);
  common::get_yaml_node("uncertainty_on", filename, uncertaintyOn);

  quadCVO_.collisionRadius = cRadius;
  quadCVO_.range = cRange;
  // quadCVO_.Ts = dt;
  quadCVO_.timeCollisionPenality = cPenalty;
  quadCVO_.uncertainPosPenality = posPenalty;
  quadCVO_.uncertainVelPenality = velPenalty;
  quadCVO_.maxAccel = maxAccel;
  quadCVO_.numPointsAdmissible = numPointsAdmissible;
  quadCVO_.order = solve_order;

  quadCVO_.bufferOn = bufferOn;
  quadCVO_.bufferPower = bufferPower;
  quadCVO_.uncertaintyOn = uncertaintyOn;

  int seed = std::chrono::system_clock::now().time_since_epoch().count();
  rng_ = std::default_random_engine(seed);
  srand(seed);
  radar_meas_ = false;
  kalman_on_ = true;
  // common::get_yaml_node("radar_measurement", filename, radar_meas_);
  // common::get_yaml_node("kalman_on", filename, kalman_on_);

  double gps_position_stdev, gps_velocity_stdev;
  gps_position_stdev = 3.0;
  gps_velocity_stdev = 1.0;
  // common::get_yaml_node("gps_position_stdev", filename, gps_position_stdev);
  // common::get_yaml_node("gps_velocity_stdev", filename, gps_velocity_stdev);

  position_dist_  = std::normal_distribution<double>(0.0, gps_position_stdev);
  velocity_dist_  = std::normal_distribution<double>(0.0, gps_velocity_stdev);
  drop_prob_ = 0; //# Values between 0 and 100, 50 from paper
  //common::get_yaml_node("drop_probability", filename, drop_prob_);
  sigmaQ_vel_ = 3;
  alphaQ_vel_ = 0.5;
  sigmaQ_jrk_ = 0.2;
  alphaQ_jrk_ = 0.5;

  sigmaR_pos_ = 3.0;
  sigmaR_vel_ = 1.0;
  sigmaR_range_ = 0.6;
  sigmaR_zenith_ = 0.01745;
  // common::get_yaml_node("sigmaQ_vel", filename, sigmaQ_vel_);
  // common::get_yaml_node("alphaQ_vel", filename, alphaQ_vel_);
  // common::get_yaml_node("sigmaQ_jrk", filename, sigmaQ_jrk_);
  // common::get_yaml_node("alphaQ_jrk", filename, alphaQ_jrk_);

  // common::get_yaml_node("sigmaR_pos", filename, sigmaR_pos_);
  // common::get_yaml_node("sigmaR_vel", filename, sigmaR_vel_);
  // common::get_yaml_node("sigmaR_range", filename, sigmaR_range_);
  // common::get_yaml_node("sigmaR_zenith", filename, sigmaR_zenith_);

  range_dist_  = std::normal_distribution<double>(0.0, sigmaR_range_);
  zenith_dist_  = std::normal_distribution<double>(0.0, sigmaR_zenith_);

  double radar_x, radar_y;
  radar_x = 0;
  radar_y = -200;//kalFilters;
  //common::get_yaml_node("radar_x", filename, radar_x);
  //common::get_yaml_node("radar_y", filename, radar_y);
  radarPos_ << radar_x, radar_y;
}

void CVOManager::get_best_vel(const vehicle::Stated& x,
  const double t,
  vehicle::Stated& xc,
  std::vector<int> visibilityList)
{

  Eigen::Vector2d av1Pos{x.p[0], x.p[1]};
  Eigen::Vector2d av1Vel{x.v[0], x.v[1]};
  Eigen::Vector2d av1VelDes{xc.v[0], xc.v[1]};
  
  std::map<int, Eigen::VectorXd> AllVehicles = env_->get_all_vehicles();
  //if (collisionList.at(0) == 1)
  //{
    // x.p*1.25;
  //  std::cout << "collision detected" << std::endl;
  //}

  std::vector<Eigen::Vector2d> inRangePos;
  std::vector<Eigen::Vector2d> inRangeVel;
  std::vector<Eigen::Vector2d> uncertVel;
  std::vector<Eigen::Vector2d> uncertPos;
  int iter = 0;
  
  // std::cout << std::endl;
  // std::cout << "vis list: " << std::endl;
  //   for (auto i: visibilityList)
  //      std::cout << i;
  // std::cout << std::endl;
  for (auto const& vehicle : AllVehicles)
  {
    // if (std::count(visibilityList.begin(), visibilityList.end(), iter))
    //  {
    //   //Also visibility list is refreshed each time so we do not need to clean here. Each set of values is new.
     
    //  continue;
    //  }
    
    //std::cout << "iter: " << iter << std::endl;
    if(visibilityList.empty())
    {}
    else if(visibilityList[iter] == 0)
    {
     //failed messages
     //std::cout << "vehicle: " << iter << " ,Failed" << std::endl;
     iter++;
     continue;
    }
    //std::cout << "vehicle: " << iter << " , Success" << std::endl;
    iter++;
    
    //successful messages
    // if( (rand() % 100) < drop_prob_)
    //   continue;
    //std::cout << "didnt jump it " <<std::endl;
    if(vehicle.first == id_)
      continue;
    
    // if (vehicleComms[id_][vehicle.first] == 1.0)
    //   continue; 
    //std::cout << "First" << std::endl;
    if(kalman_on_)
    {
      if(kalFilters.find(vehicle.first) == kalFilters.end() && //So the first part of this if statement is true
        (vehicle.second.head(2)-x.p.head(2)).norm() < quadCVO_.range)
      {
        //std::cout << "Two" << std::endl;
        kalFilters[vehicle.first] = new GenKalmanFilter(); //this is a memory leak
        VectorXd inXHat(8);
        if(radar_meas_)
        {
          Eigen::Vector2d p_hat;
          get_radar_uncertainty(vehicle.second.segment<2>(0), p_hat);
          inXHat(0) = p_hat(0);
          inXHat(1) = p_hat(1);
          inXHat(2) = 0;
          inXHat(3) = 0;
        }
        else
        {
          inXHat(0) = vehicle.second[0] + position_dist_(rng_);
          inXHat(1) = vehicle.second[1] + position_dist_(rng_);
          inXHat(2) = vehicle.second[2] + velocity_dist_(rng_);
          inXHat(3) = vehicle.second[3] + velocity_dist_(rng_);
        }
        inXHat(4) = 0;
        inXHat(5) = 0;
        inXHat(6) = 0;
        inXHat(7) = 0;
        //std::cout << "Here" << std::endl;
        kalFilters[vehicle.first]->init(sigmaQ_vel_, alphaQ_vel_,
          sigmaQ_jrk_, alphaQ_jrk_,
          sigmaR_pos_, sigmaR_vel_,
          sigmaR_range_, sigmaR_zenith_,
          CONSTANT_JERK, kalman_dt_, id_, vehicle.first, inXHat);
      }
      else if((vehicle.second.head(2)-x.p.head(2)).norm() > quadCVO_.range)
      {
        delete kalFilters[vehicle.first];
        kalFilters.erase(vehicle.first);
      }
      else
      {

        if(radar_meas_)
        {
          Eigen::Vector2d p_hat;
          get_radar_uncertainty(vehicle.second.segment<2>(0), p_hat);

          // inX(0) = p_hat(0);
          // inX(1) = p_hat(1);
          // inX(2) = 0;
          // inX(3) = 0;

          kalFilters[vehicle.first]->update_radar(p_hat, radarPos_);
        }
        else
        {
          VectorXd inX(4);
          inX(0) = vehicle.second[0] + position_dist_(rng_);
          inX(1) = vehicle.second[1] + position_dist_(rng_);
          inX(2) = vehicle.second[2] + velocity_dist_(rng_);
          inX(3) = vehicle.second[3] + velocity_dist_(rng_);
          kalFilters[vehicle.first]->update(inX); //Here is our SEG fault. Not sure why yet.
        }
      }
    }
    else
    {
      Eigen::Vector2d position{vehicle.second[0] + position_dist_(rng_), vehicle.second[1] + position_dist_(rng_)};
      Eigen::Vector2d velocity{vehicle.second[2] + velocity_dist_(rng_), vehicle.second[3] + velocity_dist_(rng_)};

      // std::cout << position << " " << velocity << "\n";
      if((vehicle.second.head(2)-x.p.head(2)).norm() < quadCVO_.range)
      {
        inRangePos.push_back(position);
        inRangeVel.push_back(velocity);
        uncertPos.push_back(Eigen::Vector2d{0.0, 0.0});
        uncertVel.push_back(Eigen::Vector2d{0.0, 0.0});
      }
    }

  }

  if(kalman_on_)
  {
    for (auto const& filter : kalFilters)
    {
      VectorXd xhat = filter.second->get_estimated_state();
      MatrixXd P = filter.second->get_covariance_matrix();
      inRangePos.push_back(Eigen::Vector2d{xhat[0], xhat[1]});
      inRangeVel.push_back(Eigen::Vector2d{xhat[2], xhat[3]});
      uncertPos.push_back(Eigen::Vector2d{pow(P(0,0),0.5), pow(P(1,1),0.5)});
      uncertVel.push_back(Eigen::Vector2d{pow(P(2,2),0.5), pow(P(3,3),0.5)});

    }
  }
  Eigen::Vector2d vel = quadCVO_.get_best_vel(av1Pos, 
    av1Vel,
    av1VelDes,
    x.toEigen(),
    dt_,
    inRangePos,
    inRangeVel,
    uncertPos,
    uncertVel);
  //vel[0] = -0.1;
  //vel[1] = 0.0;
  xc.v = Eigen::Vector3d{vel[0], vel[1], xc.v[2]};
}

// void CVOManager::get_best_vel(const vehicle::Stated& x,
//   const double t,
//   vehicle::Stated& xc,
//   uat::UATModel *model,
//   std::vector<int> visibilityList,
//   std::vector<int> collisionList)
// {
  
// //quads[i]->getState(), t, xc, model, visibilityList, collisionList

//   Eigen::Vector2d av1Pos{x.p[0], x.p[1]};
//   Eigen::Vector2d av1Vel{x.v[0], x.v[1]};
//   Eigen::Vector2d av1VelDes{xc.v[0], xc.v[1]};
  
//   std::map<int, Eigen::VectorXd> AllVehicles = env_->get_all_vehicles();
//   //if (collisionList.at(0) == 1)
//   //{
//     // x.p*1.25;
//   //  std::cout << "collision detected" << std::endl;
//   //}

//   std::vector<Eigen::Vector2d> inRangePos;
//   std::vector<Eigen::Vector2d> inRangeVel;
//   std::vector<Eigen::Vector2d> uncertVel;
//   std::vector<Eigen::Vector2d> uncertPos;
//   int iter = 0;
//   for (auto const& vehicle : AllVehicles)
//   {
//     if (std::count(visibilityList.begin(), visibilityList.end(), iter))
//       {
//       //Also visibility list is refreshed each time so we do not need to clean here. Each set of values is new.
//       continue;
//       }
//     iter++;
//     // if( (rand() % 100) < drop_prob_)
//     //   continue;
//     if(vehicle.first == id_)
//       continue;
    
//     if(kalman_on_)
//     {
//       if(kalFilters.find(vehicle.first) == kalFilters.end() && //So the first part of this if statement is true
//         (vehicle.second.head(2)-x.p.head(2)).norm() < quadCVO_.range)
//       {
//         //std::cout << "Two" << std::endl;
//         kalFilters[vehicle.first] = new GenKalmanFilter(); //this is a memory leak
//         VectorXd inXHat(8);
//         if(radar_meas_)
//         {
//           Eigen::Vector2d p_hat;
//           get_radar_uncertainty(vehicle.second.segment<2>(0), p_hat);
//           inXHat(0) = p_hat(0);
//           inXHat(1) = p_hat(1);
//           inXHat(2) = 0;
//           inXHat(3) = 0;
//         }
//         else
//         {
//           inXHat(0) = vehicle.second[0] + position_dist_(rng_);
//           inXHat(1) = vehicle.second[1] + position_dist_(rng_);
//           inXHat(2) = vehicle.second[2] + velocity_dist_(rng_);
//           inXHat(3) = vehicle.second[3] + velocity_dist_(rng_);
//         }
//         inXHat(4) = 0;
//         inXHat(5) = 0;
//         inXHat(6) = 0;
//         inXHat(7) = 0;
//         //std::cout << "Here" << std::endl;
//         kalFilters[vehicle.first]->init(sigmaQ_vel_, alphaQ_vel_,
//           sigmaQ_jrk_, alphaQ_jrk_,
//           sigmaR_pos_, sigmaR_vel_,
//           sigmaR_range_, sigmaR_zenith_,
//           CONSTANT_JERK, kalman_dt_, id_, vehicle.first, inXHat);
//       }
//       else if((vehicle.second.head(2)-x.p.head(2)).norm() > quadCVO_.range)
//       {
//         delete kalFilters[vehicle.first];
//         kalFilters.erase(vehicle.first);
//       }
//       else
//       {

//         if(radar_meas_)
//         {
//           Eigen::Vector2d p_hat;
//           get_radar_uncertainty(vehicle.second.segment<2>(0), p_hat);

//           // inX(0) = p_hat(0);
//           // inX(1) = p_hat(1);
//           // inX(2) = 0;
//           // inX(3) = 0;

//           kalFilters[vehicle.first]->update_radar(p_hat, radarPos_);
//         }
//         else
//         {
//           VectorXd inX(4);
//           inX(0) = vehicle.second[0] + position_dist_(rng_);
//           inX(1) = vehicle.second[1] + position_dist_(rng_);
//           inX(2) = vehicle.second[2] + velocity_dist_(rng_);
//           inX(3) = vehicle.second[3] + velocity_dist_(rng_);
//           kalFilters[vehicle.first]->update(inX); //Here is our SEG fault. Not sure why yet.
//         }
//       }
//     }
//     else
//     {
//       Eigen::Vector2d position{vehicle.second[0] + position_dist_(rng_), vehicle.second[1] + position_dist_(rng_)};
//       Eigen::Vector2d velocity{vehicle.second[2] + velocity_dist_(rng_), vehicle.second[3] + velocity_dist_(rng_)};

//       // std::cout << position << " " << velocity << "\n";
//       if((vehicle.second.head(2)-x.p.head(2)).norm() < quadCVO_.range)
//       {
//         inRangePos.push_back(position);
//         inRangeVel.push_back(velocity);
//         uncertPos.push_back(Eigen::Vector2d{0.0, 0.0});
//         uncertVel.push_back(Eigen::Vector2d{0.0, 0.0});
//       }
//     }

//   }

//   if(kalman_on_)
//   {
//     for (auto const& filter : kalFilters)
//     {
//       VectorXd xhat = filter.second->get_estimated_state();
//       MatrixXd P = filter.second->get_covariance_matrix();
//       inRangePos.push_back(Eigen::Vector2d{xhat[0], xhat[1]});
//       inRangeVel.push_back(Eigen::Vector2d{xhat[2], xhat[3]});
//       uncertPos.push_back(Eigen::Vector2d{pow(P(0,0),0.5), pow(P(1,1),0.5)});
//       uncertVel.push_back(Eigen::Vector2d{pow(P(2,2),0.5), pow(P(3,3),0.5)});

//     }
//   }
//   //Memory leak here
//   Eigen::Vector2d vel = quadCVO_.get_best_vel(av1Pos, 
//     av1Vel,
//     av1VelDes,
//     x.toEigen(),
//     dt_,
//     inRangePos,
//     inRangeVel,
//     uncertPos,
//     uncertVel);

//   xc.v = Eigen::Vector3d{vel[0], vel[1], xc.v[2]};
// }

void CVOManager::propagate(const double &t)
{
  for (auto const& filter : kalFilters)
  {
    filter.second->predict(t);
  }

}

void CVOManager::get_radar_uncertainty(const Eigen::Vector2d& p,
  Eigen::Vector2d& p_hat)
{
  double mav_x = p(0) - radarPos_(0);
  double mav_y = p(1) - radarPos_(1);
  double range = sqrt(pow(mav_x,2) + pow(mav_y,2));
  double zenith = atan2(mav_x, mav_y);
  double rangeHat = floor(range/sigmaR_range_)*sigmaR_range_; //range + range_dist_(rng_); //
  double zenithHat = zenith + zenith_dist_(rng_);
  p_hat << rangeHat*sin(zenithHat) + radarPos_(0), rangeHat*cos(zenithHat) + radarPos_(1);
}


} // namespace quadrotor
