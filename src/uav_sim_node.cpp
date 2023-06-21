#include <typeinfo>
#include <cmath>
#include "../lib/common_cpp/include/common_cpp/common.h"
#include "quadrotor.h"
#include "quad_control.h"
#include "../lib/common_cpp/include/common_cpp/progress_bar.h"
#include "quad_wp_manager.h"
#include "quad_cvo_manager.h"
#include "environment.h"
#include "../lib/common_cpp/include/common_cpp/logger.h"
#include <vector>
#include "vehicle.h"
#include "uatModel.h"
#include <sys/resource.h>

//namespace py = pybind11;



#include "waypoint_generator.h"
using namespace std::chrono;
// #include <experimental/filesystem>
// #include <filesystem>
// namespace fs = std::experimental::filesystem;
// Trying to find out our stack size
// size_t stackavail()
// {
//   // page range
//   MEMORY_BASIC_INFORMATION mbi;                           
//   // get range
//   VirtualQuery((PVOID)&mbi, &mbi, sizeof(mbi));           
//   // subtract from top (stack grows downward on win)
//   return (UINT_PTR) &mbi-(UINT_PTR)mbi.AllocationBase;    
// }

int main(int argc, char** argv)
{
  auto start = high_resolution_clock::now();
  const rlim_t kStackSize = 24 * 1024 * 1024;   // min stack size = 16 MB
    struct rlimit rl;
    int result;

    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0)
    {
        if (rl.rlim_cur < kStackSize)
        {
            rl.rlim_cur = kStackSize;
            result = setrlimit(RLIMIT_STACK, &rl);
            if (result != 0)
            {
                fprintf(stderr, "setrlimit returned result = %d\n", result);
            }
        }
    }
  bool use_random_seed;
  double t(0), tf, dt, cRadius, startRadius,maxVelocity;
  int numVehicles;

  // std::string uavSimPath = ros::package::getPath("uav_sim");
  common::get_yaml_node("use_random_seed", "../params/sim.yaml", use_random_seed);
  common::get_yaml_node("tf", "../params/sim.yaml", tf);
  common::get_yaml_node("dt", "../params/sim.yaml", dt);
  common::get_yaml_node("number_vehicles", "../params/sim.yaml", numVehicles);
  common::get_yaml_node("start_radius", "../params/sim.yaml", startRadius);
  common::get_yaml_node("max_velocity", "../params/sim.yaml", maxVelocity);

  common::get_yaml_node("collision_radius", "../params/cvo.yaml", cRadius);
  //numVehicles = 3;

  int seed = 0;
  if (use_random_seed){
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    //seed = M_PI;
    //seed = -949817955;//195804948;
    std::srand(seed);
    //std::srand(M_PI);
  }
  // Create progress bar
  common::ProgressBar prog_bar;
  prog_bar.init(tf/dt,40);

  // Create environment
  environment::Environment env("../params/sim.yaml");

  quadrotor::Quadrotor* quads[numVehicles];
  quadrotor::Controller* quadControllers[numVehicles];
  quadrotor::WPManager* quadWPManagers[numVehicles];
  quadrotor::CVOManager* quadCVOManagers[numVehicles];

  WaypointGenerator wyGen = WaypointGenerator(startRadius, numVehicles, cRadius, seed);
  std::vector<Eigen::Vector2d> allWaypoints = wyGen.get_even_waypoints(); //decide on what version we are doing, path wise
  //this gives the first and last waypoint as point[0] and point[1]
  int numWaypoints = allWaypoints.size()/numVehicles;

  for(int i(0); i<numVehicles; i++)
  {
    std::vector<double> waypoints;
    std::string name = "quad"+std::to_string(i);

    for(int j(0); j<numWaypoints; j++)
    {
      waypoints.push_back(allWaypoints[2*i+j](0));
      waypoints.push_back(allWaypoints[2*i+j](1));
      waypoints.push_back(0); //waypoints[numWaypoints*i+j]);
      waypoints.push_back(0); //waypoints[numWaypoints*i+j]);
    }

    quadrotor::Quadrotor* quad = new quadrotor::Quadrotor("../params/quadrotor.yaml",
        name,
        waypoints,
        env,
        use_random_seed,
        i);
    
    quads[i] = quad;
    //std::cout << "quads: "  << quads[i]->getState().v << std::endl;
    quadrotor::Controller* quadController = new quadrotor::Controller();
    quadController->load("../params/quadrotor.yaml",
        waypoints,
        use_random_seed,
        name,
        maxVelocity);
    quadControllers[i] = quadController;

    quadrotor::WPManager* quadWPManager = new quadrotor::WPManager();
    quadWPManager->load("../params/quadrotor.yaml",
        waypoints,
        use_random_seed,
        name,
        maxVelocity);
    quadWPManagers[i] = quadWPManager;

    quadrotor::CVOManager* quadCVOManager = new quadrotor::CVOManager();
    quadCVOManager->load("../params/cvo.yaml",
        &env,
        i,
        dt,
        dt*1000);
    quadCVOManagers[i] = quadCVOManager;

     
    env.initVehicle(quad->getState().p, quad->getState().v, quad->id_);
    

  }
  
  vehicle::Stated allXC[numVehicles];

  // Main simulation loop
  
  int nextUpdate = 0;
  std::set<std::pair<double,double>> set;
  std::map<std::pair<double,double>, double> map;
  
  // CPyInstance pyInstance;
  uat::UATModel* model = new uat::UATModel();
  
  model->getConst(numVehicles,tf,seed);
  model->init_vehicles();
  std::vector<int> visibilityList = std::vector<int> (1,0);
  std::vector<int> collisionList = std::vector<int> (1,0);

  int check = 0;
  while (t <= tf)
  {
    t += dt;
    prog_bar.print(int(t)/dt);
    nextUpdate += 1;

    // Run each vehicle
    int count = 0;
    for(long i(0); i<numVehicles; i++)
    {
      // std::cout << "############## Vehicle " << i << " ######################" << std::endl;
      // std::cout << "####################################" << std::endl;
      std::vector<double> coord (3);
      quadCVOManagers[i]->propagate(t);
      for (int j(0); j < 3; j++)
      {
        coord.at(j) = quads[i]->getState().p.coeffRef(j,0);
      }
      //model->updatePosition(i,coord,t); // This is called in propagate()
        if(nextUpdate >= 1000)
        {
          for (int j(0); j < 3; j++)
          {
          coord.at(j) = quads[i]->getState2().p.coeffRef(j,0);
          
          }
          model->updatePosition(i,coord,t); // This is called in propagate()
          if(count == numVehicles-1)
          {
            //std::cout << "SOme Statement" << std::endl;
            //For model2 we only update once every second. This saves power and 
            //works better with the current code. Enjoy
            //std::cout << "Time: " << t << std::endl;
            std::vector<int> visibilityList = model->propagate(i); // this causes the problem in the supercomputer
            //std::vector<int> collisionList = model->collisionTest();
            //std::cout << "i in here: " << i << std::endl;
            
            count += 1;
          }
          vehicle::Stated xc = quadWPManagers[i]->updateWaypointManager(quads[i]->getState());
          quadCVOManagers[i]->get_best_vel(quads[i]->getState(), t, xc, model, visibilityList, collisionList);
          //vehicle::Stated xc = quadWPManagers[i]->updateWaypointManager(quads[i]->getState());

          allXC[i] = xc;
          count += 1;
        }

      quadrotor::uVector u = quadControllers[i]->computeControl(quads[i]->getState(), t, allXC[i]); //returning all the throttle,yaw and 
      // std::cout << "Vehicle #: " << i << std::endl;
      // if(i == 0)
      // for(int h = 0; h < u.size(); h++)
      // {
      //   std::cout << u[h] << std::endl;
      // }
      quads[i]->run(t, env, u);
      std::vector<double> pos(3);
      std::vector<double> vec(3);
      // for (int j(0); j < 3; j++)
      // {
      //   pos.at(j) = quads[i]->getState().p.coeffRef(j,0);
      //   vec.at(j) = quads[i]->getState().v.coeffRef(j,0);
      // }
      
      //   std::cout << "Position is:" << std::endl;
      //   std::cout << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
      //   std::cout << "Velocity is:" << std::endl;
      //   std::cout << vec[0] << " " << vec[1] << " " << vec[2] << std::endl;
      //   std::cout << "####################################" << std::endl;
      
    }
    
    if(nextUpdate >= 1000) //was 1000
    {
      nextUpdate = 0;
    }

    env.updateWind(t);
    for(int i(0); i<numVehicles; i++)
    {
        env.updateVehicle(quads[i]->getState().p, quads[i]->getState().v, quads[i]->id_);
        for(int j(i); j<numVehicles; j++)
        {
          if(quads[i]->id_ == quads[j]->id_)
            continue;
          vehicle::Stated s1 = quads[i]->getState();
          vehicle::Stated s2 = quads[j]->getState();
          vehicle::dxVector s = s1 - s2;
          if(s.segment<2>(0).norm() < 2)
          {
            std::pair<double,double> pair = std::make_pair(i,j);
            set.insert(pair);
            if(map.find(pair) == map.end())
              map[pair] = s.segment<3>(0).norm();
            else if(s.segment<3>(0).norm() < map[pair])
              map[pair] = s.segment<3>(0).norm();
          }

        }
    }
  }
  
  
  long int collisionNum = set.size(); //Gets the amount of collisions.
  model->printResults(collisionNum,cRadius,maxVelocity); //Prints the results in CSV
  std::set<std::pair<double,double>>::iterator it = set.begin();
  while (it != set.end())
 {
  	std::cout << "pair: " <<  (*it).first << " , " << (*it).second << std::endl;
    std::pair<double,double> temp = *it;
    std::cout <<  "closest point: " << map[temp] << std::endl;
  	it++;
  }

  //prog_bar.finished();
  
  // ros::spin();
  std::cout << "crashes: " << collisionNum << std::endl;
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<seconds>(stop - start);
  std::cout << "is this it? " << duration.count() << std::endl;

  return 0;
}
