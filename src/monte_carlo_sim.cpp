

// /* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// /*
//  * Copyright (c) 2010,2011,2012,2013 TELEMATICS LAB, Politecnico di Bari
//  *
//  * This file is part of LTE-Sim
//  *
//  * LTE-Sim is free software; you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License version 3 as
//  * published by the Free Software Foundation;
//  *
//  * LTE-Sim is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with LTE-Sim; if not, see <http://www.gnu.org/licenses/>.
//  *
//  * Author: Giuseppe Piro <g.piro@poliba.it>
//  */
// #include "../lte-sim-dev/src/channel/include/LteChannel.h"
// //#include "../lte-sim-dev/src/componentManagers/include/NetworkManager.h"
// #include "../lte-sim-dev/src/core_lte/spectrum/include/bandwidth-manager.h"
// #include "../lte-sim-dev/src/networkTopology/include/Cell.h"
// #include "../lte-sim-dev/src/core_lte/eventScheduler/include/simulator.h"
// #include "../lte-sim-dev/src/flows/application/include/InfiniteBuffer.h"
// #include "../lte-sim-dev/src/flows/QoS/include/QoSParameters.h"
// #include "../lte-sim-dev/src/componentManagers/include/FrameManager.h"
// #include "../lte-sim-dev/src/componentManagers/include/FlowsManager.h"
// #include "../lte-sim-dev/src/device/include/ENodeB.h"
// #include "../lte-sim-dev/src/channel/include/LteChannel.h"
// #include "../lte-sim-dev/src/phy/include/enb-lte-phy.h"
// #include "../lte-sim-dev/src/phy/include/ue-lte-phy.h"
// #include "../lte-sim-dev/src/core_lte/spectrum/include/bandwidth-manager.h"
// #include "../lte-sim-dev/src/networkTopology/include/Cell.h"
// #include "../lte-sim-dev/src/protocolStack/packet/include/packet-burst.h"
// #include "../lte-sim-dev/src/protocolStack/packet/include/Packet.h"
// #include "../lte-sim-dev/src/core_lte/eventScheduler/include/simulator.h"
// #include "../lte-sim-dev/src/flows/application/include/InfiniteBuffer.h"
// #include "../lte-sim-dev/src/flows/application/include/VoIP.h"
// #include "../lte-sim-dev/src/flows/application/include/CBR.h"
// #include "../lte-sim-dev/src/flows/application/include/TraceBased.h"
// #include "../lte-sim-dev/src/device/IPClassifier/include/ClassifierParameters.h"
// #include "../lte-sim-dev/src/flows/QoS/include/QoSParameters.h"
// #include "../lte-sim-dev/src/flows/QoS/include/QoSForEXP.h"
// #include "../lte-sim-dev/src/flows/QoS/include/QoSForFLS.h"
// #include "../lte-sim-dev/src/flows/QoS/include/QoSForM_LWDF.h"
// #include "../lte-sim-dev/src/componentManagers/include/FrameManager.h"
// #include "../lte-sim-dev/src/utility/include/seed.h" 
// #include "../lte-sim-dev/src/utility/include/RandomVariable.h"
// #include "../lte-sim-dev/src/phy/include/wideband-cqi-eesm-error-model.h"
// #include "../lte-sim-dev/src/phy/include/simple-error-model.h"
// #include "../lte-sim-dev/src/channel/propagation-model/include/macrocell-urban-area-channel-realization.h"
// #include "../lte-sim-dev/src/load-parameters.h"
// #include "../lte-sim-dev/src/device/include/ENodeB.h"
#include <queue>
#include <fstream>
#include <stdlib.h>
#include <cstring>
#include <iostream>

// #include "../../channel/include/LteChannel.h"
// #include "../../core_lte/spectrum/include/bandwidth-manager.h"
// #include "../../networkTopology/include/Cell.h"
// #include "../../core_lte/eventScheduler/include/simulator.h"
// #include "../../flows/application/include/InfiniteBuffer.h"
// #include "../../flows/QoS/include/QoSParameters.h"
// #include "../../componentManagers/include/FrameManager.h"
// #include "../../componentManagers/include/FlowsManager.h"
// #include "../../device/include/ENodeB.h"
// #include "../../device/include/Gateway.h"
// #include "../../device/include/UserEquipment.h"
// #include "../../channel/include/LteChannel.h"
// #include "../../phy/include/enb-lte-phy.h"
// #include "../../phy/include/ue-lte-phy.h"
// #include "../../core_lte/spectrum/include/bandwidth-manager.h"
// #include "../../networkTopology/include/Cell.h"
// #include "../lte-sim-dev/src/protocolStack/packet/include/packet-burst.h"
// #include "../lte-sim-dev/src/protocolStack/packet/include/Packet.h"
// #include "../lte-sim-dev/src/core_lte/eventScheduler/include/simulator.h"
// #include "../lte-sim-dev/src/flows/application/include/InfiniteBuffer.h"
// #include "../lte-sim-dev/src/flows/application/include/VoIP.h"
// #include "../lte-sim-dev/src/flows/application/include/CBR.h"
// #include "../lte-sim-dev/src/flows/application/include/TraceBased.h"
// #include "../lte-sim-dev/src/device/IPClassifier/include/ClassifierParameters.h"
// #include "../lte-sim-dev/src/flows/QoS/include/QoSParameters.h"
// #include "../lte-sim-dev/src/flows/QoS/include/QoSForEXP.h"
// #include "../lte-sim-dev/src/flows/QoS/include/QoSForFLS.h"
// #include "../lte-sim-dev/src/flows/QoS/include/QoSForM_LWDF.h"
// #include "../lte-sim-dev/src/componentManagers/include/FrameManager.h"
// #include "../lte-sim-dev/src/utility/include/seed.h"
// #include "../lte-sim-dev/src/utility/include/RandomVariable.h"
// #include "../lte-sim-dev/src/utility/include/CellPosition.h"
// #include "../lte-sim-dev/src/utility/include/frequency-reuse-helper.h"
// #include "../lte-sim-dev/src/device/CqiManager/include/fullband-cqi-manager.h"
// #include "../lte-sim-dev/src/phy/include/wideband-cqi-eesm-error-model.h"
// #include "../lte-sim-dev/src/phy/include/simple-error-model.h"
// #include "../lte-sim-dev/src/channel/propagation-model/include/macrocell-urban-area-channel-realization.h"
// #include "../load-parameters.h"
// #include "../lte-sim-dev/src/channel/propagation-model/include/propagation-loss-model.h"


#include <typeinfo>
#include <cmath>
#include "../lib/common_cpp/include/common_cpp/common.h"
#include "../include/quadrotor.h"
#include "../include/quad_control.h"
#include "../lib/common_cpp/include/common_cpp/progress_bar.h"
#include "../include/quad_wp_manager.h"
#include "../include/quad_cvo_manager.h"
#include "../include/environment.h"
#include "../lib/common_cpp/include/common_cpp/logger.h"
#include <vector>
#include "../include/vehicle.h"
#include <random>
//#include "../5G-air-simulator/src/scenarios/UAV.h"
#include "../5G-air-simulator/src/channel/RadioChannel.h"
#include "../5G-air-simulator/src/channel/propagation-model/channel-realization.h"
#include "../5G-air-simulator/src/device/Gateway.h"
#include "../5G-air-simulator/src/device/UserEquipment.h"
#include "../5G-air-simulator/src/device/CqiManager/wideband-cqi-manager.h"
#include "../5G-air-simulator/src/device/CqiManager/fullband-cqi-manager.h"
#include "../5G-air-simulator/src/phy/wideband-cqi-eesm-error-model.h"
#include "../5G-air-simulator/src/channel/propagation-model/propagation-loss-model.h"

#include "../5G-air-simulator/src/scenarios/UAV.h"
#include "../5G-air-simulator/src/scenarios/single-cell-with-interference.h"
#include "../5G-air-simulator/src/scenarios/simple.h"
#include "../5G-air-simulator/src/channel/RadioChannel.h"
#include "../5G-air-simulator/src/phy/gnb-phy.h"
#include "../5G-air-simulator/src/phy/ue-phy.h"
#include "../5G-air-simulator/src/core/spectrum/bandwidth-manager.h"
#include "../5G-air-simulator/src/networkTopology/Cell.h"
#include "../5G-air-simulator/src/protocolStack/packet/packet-burst.h"
#include "../5G-air-simulator/src/protocolStack/mac/packet-scheduler/downlink-packet-scheduler.h"
#include "../5G-air-simulator/src/protocolStack/mac/AMCModule.h"
#include "../5G-air-simulator/src/protocolStack/packet/Packet.h"
#include "../5G-air-simulator/src/protocolStack/rrc/ho/handover-entity.h"
#include "../5G-air-simulator/src/protocolStack/rrc/ho/power-based-ho-manager.h"
#include "../5G-air-simulator/src/core/eventScheduler/simulator.h"
#include "../5G-air-simulator/src/flows/application/InfiniteBuffer.h"
#include "../5G-air-simulator/src/flows/application/VoIP.h"
#include "../5G-air-simulator/src/flows/application/CBR.h"
#include "../5G-air-simulator/src/flows/application/FTP2.h"
#include "../5G-air-simulator/src/flows/application/TraceBased.h"
#include "../5G-air-simulator/src/device/IPClassifier/ClassifierParameters.h"
#include "../5G-air-simulator/src/flows/QoS/QoSParameters.h"
#include "../5G-air-simulator/src/flows/QoS/QoSForEXP.h"
#include "../5G-air-simulator/src/flows/QoS/QoSForFLS.h"
#include "../5G-air-simulator/src/flows/QoS/QoSForM_LWDF.h"
#include "../5G-air-simulator/src/componentManagers/FrameManager.h"
#include "../5G-air-simulator/src/utility/RandomVariable.h"
#include "../5G-air-simulator/src/utility/UsersDistribution.h"
#include "../5G-air-simulator/src/load-parameters.h"
//#include "uatModel.h"
//#include "vehicleMSO.h"
#include <sys/resource.h>
// #include "../lte-sim-dev/src/channel/include/LteChannel.h"
// //#include "../lte-sim-dev/src/componentManagers/include/NetworkManager.h"
// #include "../lte-sim-dev/src/core_lte/spectrum/include/bandwidth-manager.h"
// #include "../lte-sim-dev/src/networkTopology/include/Cell.h"
// #include "../lte-sim-dev/src/core_lte/eventScheduler/include/simulator.h"
// #include "../lte-sim-dev/src/flows/application/include/InfiniteBuffer.h"
// #include "../lte-sim-dev/src/flows/QoS/include/QoSParameters.h"
// #include "../lte-sim-dev/src/componentManagers/include/FrameManager.h"
// #include "../lte-sim-dev/src/componentManagers/include/FlowsManager.h"
// #include "../lte-sim-dev/src/device/include/ENodeB.h"

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010,2011,2012,2013 TELEMATICS LAB, Politecnico di Bari
 *
 * This file is part of LTE-Sim
 *
 * LTE-Sim is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * LTE-Sim is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LTE-Sim; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Giuseppe Piro <g.piro@poliba.it>
 */


///########################################################################################
//Good site for LTE explainations
// http://lte-epc.blogspot.com/2013/06/lte-radio-quality-indicators.html
///########################################################################################

#include <queue>
#include <fstream>
#include <stdlib.h>
#include <cstring>
#include <iostream>


#include "waypoint_generator.h"

using namespace std::chrono;

std::mt19937 commonGen(time(NULL));


int main(int argc, char** argv)
{
  bool use_random_seed;
  double t(0), tf, dt, cRadius, startRadius,maxVelocity,cone_range;
  int numVehicles;
  float power;
  double maxAccel;
  //Starting variables that we must set
  auto start = high_resolution_clock::now();
  numVehicles = std::stoi(argv[1]);
  //radius of total simulation area
  startRadius = std::stoi(argv[2]);
  //transmission power
  power = std::stof(argv[3]);
  //Max possible velocity
  maxVelocity = std::stof(argv[4]);
  //crash radius, an area around each vehicle
  cRadius = std::stod(argv[5]);
  //range of the collision cone "looking forward"
  cone_range = std::stod(argv[6]);
	//These are to determine which simulation you want to do. Four options in total. Described in paper
  bool OVERLAP_ENHANCED = 0;
  bool COLLISION_ENHANCED = 0;
  bool BOTH_ENHANCED = 0;
  bool POSITION_CREATOR = 0;
  bool CELLULAR = 1;
  bool ADSB = 1;
  bool SINR_STATS = 1;
  //The 5G simulator senario that I created
  //suburban 0.6 400 3 10 11 32 8 2 8 1
  // int argc1 = 0;
  // string enviro = "suburban";
  // double isd = 0.6;
  // double userD = 10; 
  // int speed = 3;
  // double time1 = 10; 
  // int txMode = 11; 
  // int nbTx = 32; 
  // int nbMu = 8;
  // int rbRx = 2; 
  // int sched_type = 8; 
  // int seed1 = 1;
  // std::vector <std::pair<int,int>> communicationPair;
  //Speed direction is in radians.
  // "\t ./5G-air-simulator f5g-uc1 env isd density speed time tm nTx nMu nRx sched (seed)"
  //           "\n\t\t --> ./5G-air-simulator f5g-uc1 suburban 0.6 400 3 10 11 32 8 2 8 1"
  //Going to return a vector of pairs for which is not communicating
  //UAV_sim(argc1, enviro, isd, userD, speed, time1, txMode, nbTx, nbMu, rbRx, sched_type, seed1);
  //
	//LTE sim options
  //Read file
  //src dist sinr error
  
  
  //getting other loaded variables
  common::get_yaml_node("use_random_seed", "/home/jonathan/Desktop/5G_uav_sim/params/sim.yaml", use_random_seed);
  common::get_yaml_node("tf", "/home/jonathan/Desktop/5G_uav_sim/params/sim.yaml", tf);
  common::get_yaml_node("dt", "/home/jonathan/Desktop/5G_uav_sim/params/sim.yaml", dt);
  common::get_yaml_node("collision_acceleration","/home/jonathan/Desktop/5G_uav_sim/params/cvo.yaml",maxAccel);
  //common::get_yaml_node("number_vehicles", "../params/sim.yaml", numVehicles);
  //common::get_yaml_node("start_radius", "../params/sim.yaml", startRadius);
  //common::get_yaml_node("max_velocity", "../params/sim.yaml", maxVelocity);

  //common::get_yaml_node("collision_radius", "../params/cvo.yaml", cRadius); //what size counts as a collision.
  //common::get_yaml_node("collision_range", "../params/cvo.yaml", cone_range); //What size our collision cones are
  //common::get_yaml_node("effective_radiated_power", "../params/sim.yaml", power); //radiated power for each uav, options are 0.01,0.05,0.1,1.0,10.0,20.0
	
	//Random number generators:
  int seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::mt19937_64 generator (seed);
  std::uniform_real_distribution<double> dis(0.0, 1.0); //for our regular random numbers
  std::uniform_int_distribution<int> uni{0, 3601}; //for our initial frame.
  //Our power libraries, they dont change so it works well for them to be included this way. Makes compilation long though. 
  int MAX_dist = 0;
  float const *ERP_power;
  // //choose the proper power list:
  // //ERP_power will contain a pointer to the correct list of values. 
  // //This is a dictionary to the correct power tranmission. This determines how much power reaches the different vehicles through natural 
  // //LoS propogation
  if(power < 0.05)
  {
    
    float ERP_power_hundreth[3446];
    
    std::fstream myfile("../WattValues/hundreth_watt.txt", std::ios_base::in);
    int count = 0;
    float a = 0;
    std::string line;
    while(getline(myfile,line))
    {
    std::stringstream   linestream(line);
    std::string         value;

      while(getline(linestream,value,','))
      {
        a = stof(value);
        ERP_power_hundreth[count] = a;
      }
    count += 1;
    }
    ERP_power = ERP_power_hundreth;
    MAX_dist = 3446;
  }
  else if(power < 0.1)
  {
    float ERP_power_twentieth[7705];
    std::fstream myfile("../include/WattValues/twentieth_watt.txt", std::ios_base::in);
    int count = 0;
    float a = 0;
    std::string line;
    while(getline(myfile,line))
    {
    std::stringstream   linestream(line);
    std::string         value;

      while(getline(linestream,value,','))
      {
        a = stof(value);
        ERP_power_twentieth[count] = a;
      }
    count += 1;
    }
    ERP_power = ERP_power_twentieth;
    MAX_dist = 7705;
  }
  else if(power < 1.0)
  {
    float ERP_power_tenth[10897];
    std::fstream myfile("../include/WattValues/tenth_watt.txt", std::ios_base::in);
    int count = 0;
    float a = 0;
    std::string line;
    while(getline(myfile,line))
    {
    std::stringstream   linestream(line);
    std::string         value;

      while(getline(linestream,value,','))
      {
        a = stof(value);
        ERP_power_tenth[count] = a;
      }
    count += 1;
    }
    ERP_power = ERP_power_tenth;
    MAX_dist = 10897;
  }
  else if(power < 10.0)
  {
    
    float ERP_power_one[34457];
    std::fstream myfile("../include/WattValues/one_watt.txt", std::ios_base::in);
    int count = 0;
    float a = 0;
    std::string line;
    while(getline(myfile,line))
    {
    std::stringstream   linestream(line);
    std::string         value;

      while(getline(linestream,value,','))
      {
        a = stof(value);
        ERP_power_one[count] = a;
      }
    count += 1;
    }
    ERP_power = ERP_power_one;
    MAX_dist = 34457;
  }
  else if(power < 20.0)
  {
    float ERP_power_ten[74081];
    std::fstream myfile("../include/WattValues/ten_watt.txt", std::ios_base::in);
    int count = 0;
    float a = 0;
    std::string line;
    while(getline(myfile,line))
    {
    std::stringstream   linestream(line);
    std::string         value;

      while(getline(linestream,value,','))
      {
        a = stof(value);
        ERP_power_ten[count] = a;
      }
    count += 1;
    }
    ERP_power = ERP_power_ten;
    MAX_dist = 74081;
  }
  else if(power == 20.0)
  {
    float ERP_power_twenty[74081];
    std::fstream myfile("../include/WattValues/twenty_watt.txt", std::ios_base::in);
    int count = 0;
    float a = 0;
    std::string line;
    while(getline(myfile,line))
    {
    std::stringstream   linestream(line);
    std::string         value;

      while(getline(linestream,value,','))
      {
        a = stof(value);
        ERP_power_twenty[count] = a;
      }
    count += 1;
    }
    ERP_power = ERP_power_twenty;
    MAX_dist = 74081;
  }
  else
  {
	//Error
    float ERP_power = {0};
    std::cout << "Power dict problem! Does not match our areas." << std::endl; //our error
  }
  // Create progress bar
  common::ProgressBar prog_bar;
  prog_bar.init(tf/dt,40);

  // Create environment
  environment::Environment env("/home/jonathan/Desktop/5G_uav_sim/params/sim.yaml");

  // setting up the different classes to take our aircraft and their controllers.
  quadrotor::Quadrotor* quads[numVehicles];
  quadrotor::Controller* quadControllers[numVehicles];
  quadrotor::WPManager* quadWPManagers[numVehicles];
  quadrotor::CVOManager* quadCVOManagers[numVehicles];
  //Create User Equipment
  
	//starting up the waypoint generator, setting all the waypoints in the enviroment.
  WaypointGenerator wyGen = WaypointGenerator(startRadius, numVehicles, cRadius, seed);
  std::vector<Eigen::Vector2d> allWaypoints = wyGen.get_even_waypoints(); //decide on what version we are doing, path wise
  //this gives the first and last waypoint as point[0] and point[1]
  int numWaypoints = allWaypoints.size()/numVehicles;
  //for each vehicle we need to set them with a controller and waypoints. 
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
    quadrotor::Quadrotor* quad = new quadrotor::Quadrotor("/home/jonathan/Desktop/5G_uav_sim/params/quadrotor.yaml",
        name,
        waypoints,
        env,
        use_random_seed,
        i);
    
    quads[i] = quad;
	//set controller
    quadrotor::Controller* quadController = new quadrotor::Controller();
    quadController->load("/home/jonathan/Desktop/5G_uav_sim/params/quadrotor.yaml",
        waypoints,
        use_random_seed,
        name,
        maxVelocity);
    quadControllers[i] = quadController;
    quadrotor::WPManager* quadWPManager = new quadrotor::WPManager();
    quadWPManager->load("/home/jonathan/Desktop/5G_uav_sim/params/quadrotor.yaml",
        waypoints,
        use_random_seed,
        name,
        maxVelocity);
    quadWPManagers[i] = quadWPManager;
    quadrotor::CVOManager* quadCVOManager = new quadrotor::CVOManager();
    quadCVOManager->load("/home/jonathan/Desktop/5G_uav_sim/params/cvo.yaml",
        &env,
        i,
        dt,
        dt*100,cone_range,maxAccel);
    quadCVOManagers[i] = quadCVOManager;

     //Setting our position state
    env.initVehicle(quad->getState().p, quad->getState().v, quad->id_);
    std::vector<double> VehPos (3);
    std::vector<double> VehVel (3);
    //Here to make our position CSV, need a last entry for vehicle 0
    for (int j(0); j < 3; j++)
        {
        VehPos.at(j) = quad->getState().p.coeffRef(j,0);  //get our current UAV position.
        VehVel.at(j) = quad->getState().v.coeffRef(j,0);
        }
        //dist = pow(x,2) + pow(y,2);
        //          dist = sqrt(dist);
    // int posX_ue = 40; //m
    // int posY_ue = 0;  //m
    // int speed = sqrt(pow(VehVel[0],2) + pow(VehVel[1],2));
    // int speeDirection = 0;
    //ues[i] = networkManager->CreateUserEquipment (i, posX_ue, posY_ue, speed, speeDirection, cell, enb);
    //ues[i] = networkManager->CreateUserEquipment (i, VehPos[0], VehPos[1], speed, speeDirection, cell, enb);
    

  }
  //Setting our initial frames for our UATs
  for(int i = 0; i < numVehicles; i++)
  {
     //random between 0 and 3601 for our frame 
    int random_num = uni(generator);//rand() % 3601;
    //making us start at zero time for our frame
    quads[i]->initMSO(random_num,i);  
    //quads[i]->initMSO(0,i);  
  }
  
  vehicle::Stated allXC[numVehicles];

  //Our counters for collisions, have to be long so there is no overflow:
  long int chance = 0;
  long int MTL = 0;
  long int messageOverlap = 0;
  long int collision = 0;
  long int closeness = 0;
  long int CellFailures = 0;
  std::vector<int> All_MSO (3200,0);

  // Main simulation loop
  
  int nextUpdate = 0;
  std::set<std::pair<double,double>> set;
  std::map<std::pair<double,double>, double> map;
  

  //Setting our initial list to be empty for each vehicle, we will load it with the solutions later.
  std::vector<std::vector<int>> visibilityList; //std::vector<int> (0,0);
  //std::vector<std::vector<int>> visibilityListCell;
  vector<bool> visibilityListCell(numVehicles, 0); //the messages are only between the vehicle and tower. So only 1D needed

  for(int i = 0; i < numVehicles; i++)
  {
    std::vector<int> vec(0,0);
    visibilityList.push_back(vec);
  }

  std::vector<int> collisionList = std::vector<int> (0,0);
  int num_mso_total = 0;
  int check = 0;
  //std::vector<double> averageVel(int(tf),0);
  std::vector<double> velocities(numVehicles,0);
  //Main simulation for each time step (dt) and going until we reach the final time (tf)
  while (t <= tf)
  {
    
    nextUpdate += 1;

    // Run each vehicle
    
    int count = 0;
    for(int i(0); i<numVehicles; i++)
    {
      int count_Coulton = 0;
      quadCVOManagers[i]->propagate(t);
      
        if(nextUpdate <= 1) // at the start of the second
        { //once every second 
        
        //Here to make our position CSV, need a last entry for vehicle 0
          if (POSITION_CREATOR == true)
          {
            std::vector<double> coord_dist_1 (3);
            for (int j(0); j < 3; j++)
              {
              coord_dist_1.at(j) = quads[i]->getState().p.coeffRef(j,0);  //get our current UAV position.
              
              }
            std::ofstream pos("Positions.csv" , std::ios::app );
            pos << i << "," << coord_dist_1[0] << "," << coord_dist_1[1] << '\n';
            pos.close();
          }
		//for each vehicle.
        if(count == numVehicles - 1)
        { 
           

            
            //This is a vector of a pair of vectors. the inside vectors will contain 
            //distances and vehicle numbers respectively. The index of the containing vector
            //is the recieving vehicle number.

             //5G sim start
            //need positions and velocities along with directions:
            if(CELLULAR == true)
            {
              // std::cout << "#################################################" << std::endl;
              // std::cout << "This needs to run every second" << std::endl;
              // std::cout << "time: " << t << std::endl;
              // std::cout << "#################################################" << std::endl;
              //holding vectors for holding
            std::vector<double> coord_dist (3);
            std::vector<double> vel_dist (3);
            //vectors for input params for 5G sim
            std::vector<double> xpos (0); 
            std::vector<double> ypos (0); 
            std::vector<int> speed (0);
            std::vector<double> speedDir (0);  
            //get our current position for this quadrotor:
            for (int k(0); k < numVehicles; k++)
            {
              for (int j(0); j < 3; j++)
              {
              coord_dist.at(j) = quads[k]->getState().p.coeffRef(j,0);  //get our current UAV position.
              vel_dist.at(j) = quads[k]->getState().v.coeffRef(j,0); //Get current velocity
              }
              // std::cout << "our Velocities: " << vel_dist[0] << ", " << vel_dist[1] << ", " << vel_dist[2] << std::endl;
              // std::cout << "vehicle: " << k << std::endl;

              xpos.push_back(coord_dist[0]);
              ypos.push_back(coord_dist[1]);
              //multiply by 3.6 to get km/hr from m/s
              double rawspeed = sqrt(pow(vel_dist[0]*3.6,2) + pow(vel_dist[1]*3.6,2));
              //Rounding:
              //Which one should we round to?
              std::vector<double> rounding (3,0);
              std::vector<int> finalRound {3,30,120};
              rounding[0] = std::abs(rawspeed - 3);
              rounding[1] = std::abs(rawspeed - 30);
              rounding[2] = std::abs(rawspeed - 120);
              //find the smallest element:
              std::vector<double>::iterator v = std::min_element(rounding.begin(), rounding.end());
              //get our value for what it should be for the speed
              int val = finalRound[std::distance(rounding.begin(),v)];
            

              speed.push_back(val); // only 3, 30 and 120 are defined in channel-realization.cpp
              speedDir.push_back(tan(vel_dist[1]/vel_dist[0])); //convert our speed into radians to get our direction.

            }
              //Simple();
              double radius = startRadius/1000;
              int nbUE = numVehicles;
              //need to set number of Cells, in order to get our indexs right
              int nbCells = 7;
              int successCount [numVehicles] = {};
              SingleCellWithInterference (radius, nbUE, xpos, ypos ,speed, speedDir, seed, nbCells);
              std::string line;
              std::ifstream inputFile("/home/jonathan/Desktop/5G_uav_sim/src/output.txt");
              while (std::getline(inputFile, line))
              {
                  std::istringstream iss(line);
                  int a, b, d;
                  char one,two,three,four,five,six;
                  float c, e, f, g;
                  //std::cout << "Is this working inside? " << std::endl;
                  //NEED TO SET HEIGHT IN USEREQUIPMENT.CPP WHICH IS INSIDE OF DEVICE
                  if (!(iss >> a >> one >> b >> two >> c >> three >> d >> four >> e >> five >> f >> six >> g)) { break; } // error
                  //std::cout << "src: " << a << ", dist: " << b - nbCells << ", SINR: " << c << " ERROR: " << d << "X: " << e << "Y: " << f << "Z: " << g << std::endl;
                  if(d = 0) //Each message needs to reach each vehicle twice. Once to send its values. next to recieve others values.
                      successCount[b - nbCells] += 1; //because the application ID's count the cells first, then UE's
                  if(successCount[b - nbCells] > 1) //0 is a successful message
                  {
                    visibilityListCell[b - nbCells] = 1; //not in my world
                    //Need to set the vehicle as not needing to send ADSB
                    quads[b - nbCells]->setMessageSuccess(1);
                  }
                  // if(d = 1) //unsuccessful message
                  // {

                  // }
                  if(SINR_STATS = true)
                  {
                  //Vehicle, SINR, Error
                  std::ofstream pos("SINR.csv" , std::ios::app );
                  pos << b << "," << c << "," << d << '\n';
                  pos.close();
                  }
                  //then we load into the proper place in the matrix, along with the SINR
                  
              }
              //Count how many failures Cell had
              for(int k = 0; k < numVehicles; k++)
              {
                if (successCount[k] < 2)
                  CellFailures += numVehicles; //if it fails then every other vehicle cannot see it. This is to match it 
                  //with adsb. Correct or not, I dunno. Ask Ethan about this.
              }
              //Erase the data file
              std::ofstream ofs;
              ofs.open("/home/jonathan/Desktop/5G_uav_sim/src/output.txt", std::ofstream::out | std::ofstream::trunc);
              ofs.close();
            }
            if (ADSB == true)
            {
            //   std::cout << "#################################################" << std::endl;
            // std::cout << "Does ADSB run every second?" << std::endl;
            
            //   std::cout << "time: " << t << std::endl;
            //   std::cout << "#################################################" << std::endl;
            std::vector<std::vector<int>> sec; 
            std::vector<int> distance;
            std::vector<int> mso_list;
            std::vector<std::vector<std::pair<int,int>>> distanceDict;
            std::vector<float> recieved_powerDict;
            std::vector<std::vector<bool>> successful_decodeDict;
            // std::vector<std::vector<bool>> successful_decodeDict_Cell; 
            
            //std::vector<std::pair<int,std::vector<int>>> sorted_list;

            std::vector<std::pair<int,int>> sorted_list;
            for(int j = 0; j < numVehicles; j++)
            {
				        //I think even with Cellular we still have to generate the MSO each second. But it just would not transmit.
                //This is to do with the frame numbers and the previous random number being used.
                //Here we generate the MSO for each vehicle
                quads[j]->updateMSO();

                int mso_saved = quads[j]->full_mso_range();
                num_mso_total += 1;

                mso_list.push_back(mso_saved);
				        //we save the MSO in case we want to look at the distribution later.
                // std::ofstream pos("MSO.csv" , std::ios::app);
                // pos << t << "," << mso_saved << "," << j << '\n';
                // pos.close();
                std::vector<int> index;
                //std::vector<bool> MTLcounter;
                std::vector<bool> MTLcounter(numVehicles,1); // make them all false? 
                std::vector<std::pair<int,int>> distances; // This contains
                std::vector<double> coord_dist_10 (3);
                for (int g(0); g < 3; g++)
                {
                coord_dist_10.at(g) = quads[j]->getState().p.coeffRef(g,0);  //get our current UAV position.
                }
                for(int k = (j+1); k <= (j + numVehicles); k++)
                {
                  
                  std::vector<double> coord_dist_other (3);
                  for (int g(0); g < 3; g++)
                  {
                  coord_dist_other.at(g) = quads[(k % numVehicles)]->getState().p.coeffRef(g,0);
                  }
                  //Now get the distance between them:
                  double x = coord_dist_other[0] - coord_dist_10[0]; //x position
                  double y = coord_dist_other[1] - coord_dist_10[1]; //y position
                  double dist;
                  int distance = 0;
                  dist = pow(x,2) + pow(y,2);
                  dist = sqrt(dist);
                  distance = std::round(dist); //rounding to convert to the correct int.
                  int mso = quads[(k % numVehicles)]->getMSO();
                  float recieved_power = -100;
                  //go over each distance to see if our transmission is successful
                  if(distance <= 1)
                  {
                    distance = 1; //no zero distances. 
                  }
				            //loading in our power dictionairies
                  if(distance < MAX_dist)
                  {
                    recieved_power = ERP_power[distance-1]; 
                  }
                  else
                  {
                    recieved_power = -94; //Power is below the MTL
                  }
                  double random = dis(generator);
                  bool meets_minimum_trigger_level = true; //initialize the boolean
				          //Checking if our power is even in range
                  if(recieved_power >= -90.0)
                  {
                    if(j == (k % numVehicles))
                    {
                      //meets_minimum_trigger_level = false;
                    }
                    else if(random < 0.99)
                    {
                      meets_minimum_trigger_level = true;
                    }
                    else
                    {
                      meets_minimum_trigger_level = false;
                      chance += 1;
                    }
                  }
                  else if(recieved_power < -90.0 && -93 <= recieved_power) //-93.0 is reciever minimum trigger level, it can change.
                  {

                      if(j == (k % numVehicles))
                      {
                        //meets_minimum_trigger_level = false;
                      }
                      else if(random < 0.9)
                        meets_minimum_trigger_level = true;
                      else
                      {
                      meets_minimum_trigger_level = false;
                      chance += 1;
                      }
                  }
                  else
                  {
                    //distance to our own vehicle should be 1, so we will have no issues with MTL
                    meets_minimum_trigger_level = false;
                    MTL += 1;
                  }
                
                index.push_back((k % numVehicles));
                MTLcounter[(k%numVehicles)] = meets_minimum_trigger_level;
                recieved_powerDict.push_back(recieved_power);
                distances.push_back(std::make_pair(distance,(j)));
                
                }
              
              distanceDict.push_back(distances);
			  //loading the correct size for our successful conditions
              successful_decodeDict.push_back(MTLcounter);

              //successful_decodeDict_test.push_back(MTLcounter);
              sec.push_back(mso_list);
			  //increase the frame after each second
              quads[j]->increaseFrame();
			  //making a pair so we can sort it later based on mso
              if(quads[j]->getMessageSuccess() == false)
                sorted_list.push_back(std::make_pair(mso_saved,j));
              
             
            }
            
              for(int k = 0; k < numVehicles; k++)
              {
                successful_decodeDict[k][k] = 0;
                //Set our own vehicles so they cannot transmit to themselves
              }
               //Closeness Calcualtion section
            //##############################################################################################
            //std::cout << "MTL/chance failure count: " << failure_count << std::endl;
            for(int k = 0; k < numVehicles; k++)
            {
              if(quads[k]->getMessageSuccess() == false)
              {
              
                int k_mso = quads[k]->getMSO();
                for(int g = 0; g < numVehicles; g++)
                {
                  int g_mso = quads[g]->getMSO();
                  
                  if(k == g){continue;}//if its the same vehicle then do nothing.
                  else if ((k_mso - 9) <= (g_mso) && (g_mso) <= (k_mso + 8))
                    {
                    if(successful_decodeDict[k][g] == true) //was [k][g]
                      {
                        //check to see if this matches collision
                      successful_decodeDict[k][g] = false;
                      closeness += 1;
                        if (POSITION_CREATOR == true)
                          {
                            std::vector<double> coord_dist_new (3);
                            std::vector<double> coord_dist_other (3);

                            //Here to make our position CSV, need a last entry for vehicle 0
                            for (int z(0); z < 3; z++)
                              {
                              coord_dist_new.at(z) = quads[k]->getState().p.coeffRef(z,0);  //get our current UAV position.
                              coord_dist_other.at(z) = quads[g]->getState().p.coeffRef(z,0);  //get our current UAV position.

                              
                              }
                            std::ofstream pos("Collisions.csv" , std::ios::app );
                            pos << k << "," << g << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << "," << t << "," << "Closeness" << '\n';
                            pos.close();
                          }
                      }
                    }
                }
            }
            }
             //End Closeness Calcualtion section
            //##############################################################################################
            //Overlap Calcualtion section
            //##############################################################################################
			if(OVERLAP_ENHANCED = false)
			{
				std::sort(sorted_list.begin(),sorted_list.end()); //First is the mso, second is the vehicle number
				int previousMSO = sorted_list[0].first;
				int num_consecutive = 0;
				for(int k = 1; k < sorted_list.size();k++)
				{
				if(sorted_list[k].first == (previousMSO + 1) || (sorted_list[k].first == previousMSO))
				{
					if(sorted_list[k].first == (previousMSO + 1))
					{
						num_consecutive += 1;
					}
					if(num_consecutive % 2)
						for(int g = 0; g < numVehicles; g++)
						{
						if(successful_decodeDict[sorted_list[k].second][g] == true)
						{
							successful_decodeDict[sorted_list[k].second][g] = false;
							messageOverlap += 1;
							if (POSITION_CREATOR == true)
							{
								std::vector<double> coord_dist_new (3);
								std::vector<double> coord_dist_other (3);

								//Here to make our position CSV, need a last entry for vehicle 0
								for (int z(0); z < 3; z++)
									{
									coord_dist_new.at(z) = quads[sorted_list[k].second]->getState().p.coeffRef(z,0);  //get our current UAV position.
									coord_dist_other.at(z) = quads[g]->getState().p.coeffRef(z,0);  //get our current UAV position.

									
									}
								std::ofstream pos("Collisions.csv" , std::ios::app );
								pos << k << "," << g << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << "," << t << "," << "Overlap" <<'\n';
								pos.close();
							}
						}
						}
				}
				else
					num_consecutive = 0;
				previousMSO = sorted_list[k].first;
				}
			}
            //End Overlap section
            //##############################################################################################
            std::vector<std::pair<int,std::vector<int>>> collisionObjects; //to collect all our collisions in. Put in our mso and vehicle number. 
            std::vector<int> vehiclesInvolved;
            std::vector<std::vector<int>> totalVehiclesInvolved;
            std::vector<std::pair<int,int>> transmissions;
            //Get our Transmitting Vehicles!
            int collision_count = 0;
            for(int k =752; k < 3952;k++) //go through all msos
            {
              //need to count occurances in our vector of vector ints:
              int coll_count = 0;
              for(int ele : sec[numVehicles-1])
              {
                if (ele  == k)
                  coll_count += 1; //Counting occurances of that particular MSO as given by k
              }
              if(coll_count > 1) //if its more than one then we have a collision
              {
                vehiclesInvolved.clear();
                for(int g = 0; g < numVehicles; g++) //check all our vehicles
                { 
                  if(quads[g]->getMessageSuccess() == false) //only if the message has not been already recieved through cellular
                  if(quads[g]->getMSO() == k) 
                  {
                    //std::cout << "Vehicles Invoved: " << g << std::endl;
                    vehiclesInvolved.push_back(g); //Transmitting Vehicles
                  }
                }
                collisionObjects.push_back(std::make_pair(k,vehiclesInvolved)); //mso and vector of vehicle identifiers
                totalVehiclesInvolved.push_back(vehiclesInvolved);
                //std::cout << "Collision Objects mso: " << k << std::endl;
              }
              
            }
            for(int k = 0; k < collisionObjects.size();k++) //k is the mso for the first one
            {
              int mso = collisionObjects[k].first;
              
              for(int g=0;g < numVehicles; g++) // for every vehicle
              {
                if(quads[g]->getMessageSuccess() == false)
                {
                int mso_to_check = quads[g]->getMSO();
                //std::cout << "mso_to_check: " << mso_to_check << std::endl;
                //that is not part of the collision (and not switching modes)
                if (mso_to_check < (mso - 9) || (mso_to_check > (mso + 8)))
                {
                  transmissions.clear(); //make it clean
                  for(int h = 0; h < collisionObjects[k].second.size(); h++) //vehicles involved, Collision Objects contain all the vehicles involved.
                  { 
                        transmissions.push_back(std::make_pair(distanceDict[collisionObjects[k].second[h]][g].first,collisionObjects[k].second[h]));
                  }
                  std::sort(transmissions.begin(),transmissions.end());
                  float recieved_power;
                  float recieved_power_2;
                  if(transmissions[0].first > MAX_dist)
                  {
                    recieved_power = -94;
                  }
                  else  
                    float recieved_power = ERP_power[transmissions[0].first];
                  //If the transmission from the closest vehicle meets the MTL
                  if(transmissions.size() > 1)
                  {
                    if(transmissions[1].first > MAX_dist)
                    {
                      float recieved_power_2 = -94;
                    }
                    else
                    float recieved_power_2 = ERP_power[transmissions[1].first];
                  if(recieved_power >= -93.0) //93 is the minimum_trigger_level
                  {
					          int trans_power;
                    //  IMPORTANT
                    //  The "3000" is the difference between this file and main_sim_collision_enhanced.py. Change the
                    //  3000 to p.power_difference_decibel to get collision enhanced model.
                    //  If the power difference between the closest and second-closest vehicle is greater than or
                    //  equal to the necessary power difference
                  if(COLLISION_ENHANCED || BOTH_ENHANCED == true)
                  {trans_power = 4;}
                  else{trans_power = 3000;}
                      if(recieved_power - recieved_power_2 >= trans_power) //or 4, or 3000
                      {
                        int true_count = 0;
                        std::vector<int> indices;
                        // Then the closest message succeeds and the rest do not
                        for(int y = 0; y < transmissions.size() - 1;y++)
                        {
                          if(successful_decodeDict[transmissions[y+1].second][g] == true) 
                          {
                            true_count += 1;
                            indices.push_back(transmissions[y+1].second);
                          }
                          if(true_count > 1)
                          {
                            if(successful_decodeDict[indices[y]][g] == true)
                            {
                              successful_decodeDict[transmissions[y+1].second][g] = false;
                              collision += 1;
                            if(POSITION_CREATOR == true)
                            {
                              std::vector<double> coord_dist_new (3);
                              std::vector<double> coord_dist_other (3);

                              //Here to make our position CSV, need a last entry for vehicle 0
                                for (int z(0); z < 3; z++)
                                    {
                                    coord_dist_new.at(z) = quads[g]->getState().p.coeffRef(z,0);  //get our current UAV position.
                                    coord_dist_other.at(z) = quads[transmissions[y+1].second]->getState().p.coeffRef(z,0);  //get our current UAV position.

                                    
                                    }

                                std::ofstream pos("Collisions.csv" , std::ios::app );
                                pos << k << "," << g << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << "," << t <<  "," << "Collision" << '\n';
                                pos.close();
							                }
                            }
                          }
                        }

                      } 
                      else if (10 < ERP_power[transmissions[1].first] < -93.0) 
                      {
                        int true_count = 0;
                        std::vector<int> indices;
                        // Otherwise, if the second closest doesn't meet the MTL, the closest message still succeeds
                        // because we already determined that the closest meets the MTL
                        for(int y = 0; y < transmissions.size() - 1;y++)
                        {
                          if(successful_decodeDict[transmissions[y+1].second][g] == true)
                          {
                            true_count += 1;
                            indices.push_back(transmissions[y+1].second);
                          }
                          if(true_count > 1)
                          {
                            if(successful_decodeDict[indices[y]][g] == true)
                            {
                              successful_decodeDict[transmissions[y+1].second][g] = false;
                              collision += 1;
                              if(POSITION_CREATOR == true)
                            {
                              std::vector<double> coord_dist_new (3);
                              std::vector<double> coord_dist_other (3);

                              //Here to make our position CSV, need a last entry for vehicle 0
                              for (int z(0); z < 3; z++)
                                  {
                                  coord_dist_new.at(z) = quads[transmissions[y+1].second]->getState().p.coeffRef(z,0);  //get our current UAV position.
                                  coord_dist_other.at(z) = quads[g]->getState().p.coeffRef(z,0);  //get our current UAV position.

                                  
                                  }

                              std::ofstream pos("Collisions.csv" , std::ios::app );
                              pos << k << "," << g << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << "," << t << "," << "Collision" <<'\n';
                              pos.close();
							                 }
                            }
                          }
                        } 

                          
                      }
                      else
                      {
                        // The only other option is that the closest and second-closest meet the MTL, but the difference
                        // in power is not great enough for the closest to be selected, so all messages fail
                        int true_count = 0;
                        std::vector<int> indices;
                        for(int y = 0; y < transmissions.size();y++)
                         {
                         if(successful_decodeDict[transmissions[y].second][g] == true)
                          {
                            true_count += 1;
                            indices.push_back(transmissions[y].second);
                          }
                         }
                         if(true_count > 1)
                         {
                            for(int y = 0; y < indices.size();y++)
                            {
                              if(successful_decodeDict[indices[y]][g] == true)
                              {
                              successful_decodeDict[indices[y]][g] = false;
                              collision += 1;
                                if(POSITION_CREATOR == true)
                              {
                              std::vector<double> coord_dist_new (3);
                              std::vector<double> coord_dist_other (3);

                              //Here to make our position CSV, need a last entry for vehicle 0
                              for (int z(0); z < 3; z++)
                                  {
                                  coord_dist_new.at(z) = quads[indices[y]]->getState().p.coeffRef(z,0);  //get our current UAV position.
                                  coord_dist_other.at(z) = quads[g]->getState().p.coeffRef(z,0);  //get our current UAV position.

                                  
                                  }

                              std::ofstream pos("Collisions.csv" , std::ios::app );
                              pos << k << "," << g << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << "," << t << "," << "Collision" << '\n';
                              pos.close();
							                }
                              }
                            }
                            
                         } 

                         }
                  
                  }
                  else
                  {
                    //If the closest transmission doesn't meet MTL, they all fail
                    int true_count = 0;
                    std::vector<int> indices;
                    for(int y = 0; y < transmissions.size();y++) // make them all False
                       {
                        if(successful_decodeDict[transmissions[y].second][g] == true)
                          {
                            true_count += 1;
                            indices.push_back(transmissions[y].second);
                          }
                        }
                         if(true_count > 1)
                         {
                            for(int y = 0; y < indices.size();y++)
                            {
                              if(successful_decodeDict[indices[y]][g] == true)
                              {  
                                successful_decodeDict[indices[y]][g] = false;
                                collision += 1;
                              if(POSITION_CREATOR == true)
                              {
                                
                                std::vector<double> coord_dist_new (3);
                                std::vector<double> coord_dist_other (3);

                                //Here to make our position CSV, need a last entry for vehicle 0
                                for (int z(0); z < 3; z++)
                                    {
                                    coord_dist_new.at(z) = quads[indices[y]]->getState().p.coeffRef(z,0);  //get our current UAV position.
                                    coord_dist_other.at(z) = quads[g]->getState().p.coeffRef(z,0);  //get our current UAV position.
                                    }
  
                                std::ofstream pos("Collisions.csv" , std::ios::app );
                                pos << k << "," << g << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << "," << t << "," << "Collision" <<'\n';
                                pos.close();
								              }
                              }
                            }
                            
                         } 

                  }
                }
                }
               } //for cellular check
              }
              
            }
			//Now we have to load in our successful transmissions into the visibility list so we can know 
			//which vehicles are visible to who.
            for(int k = 0; k < successful_decodeDict.size(); k++)
            {
              for(int j = 0; j < successful_decodeDict[k].size();j++)
              {
                if(successful_decodeDict[k][j] == 0)
                {
                  //std::cout <<"1" << std::endl;
                  visibilityList[k].push_back(0); //whose messages have failed. 
                  //std::cout << "WHich vehicle cant be seen: " << successful_decodeDict[k][j] << std::endl;
                  //so vehicle k cannot see vehicle j if it is in this list.
                }
                else if(successful_decodeDict[k][j] == 1)
                {
                  visibilityList[k].push_back(1);
                  
                }
              }
            }
            //std::cout << "successful Decode count: " << count_decode_dict << std::endl;
          } //END of ADSB
            count += 1;

          }
          for(int k = 0;k < numVehicles;k++)
          {
            if(quads[k]->getMessageSuccess() == true) //we passed the message successfully through cellular
            {
              
            }
          }
          if(ADSB == false)
          {
            if(CELLULAR = false)
            {
            //load visibilitylist with all successes
              for(int k = 0; numVehicles; k++)
              {
                for(int j = 0; j < numVehicles; j++)
                {
                    visibilityList[k].push_back(1);
                }
              }
            }
          }
          
		      //HERE we load in our solutions that we got in the ADS-B section so we can determine our collision avoidance 
          vehicle::Stated xc = quadWPManagers[i]->updateWaypointManager(quads[i]->getState());
          //quadCVOManagers[i]->get_best_vel(quads[i]->getState(), t, xc, visibilityList[i]);
          quadCVOManagers[i]->get_best_vel(quads[i]->getState(), t, xc, visibilityList[i]);
          allXC[i] = xc;
          //##################################################
          //average Velocities this second:
          //##################################################
          velocities[i] = sqrt(pow(xc.v[0],2) + pow(xc.v[1],2)); //our average velocity
          //std::cout << "vel0: " << xc.v[0] << " vel1: " << xc.v[1] << " vel[2]:  " << xc.v[2] << std::endl;
          count += 1;
          visibilityList[i].clear();
          
        }
      
      quadrotor::uVector u = quadControllers[i]->computeControl(quads[i]->getState(), t, allXC[i]); //returning all the throttle,yaw and 
      quads[i]->run(t, env, u);
    }
    // std::cout << "######################################" << std::endl;
    // std::cout << "So does our time increase at all?" << std::endl;
    // std::cout << "time: " << t << std::endl;
    // std::cout << "######################################" << std::endl;
    t += dt;
    prog_bar.print(int(t)/dt);
    //nextUpdate += 1;
	//Next update will change based on time step. So a time step of 0.01 will have 100 steps before another second.
    if(nextUpdate >= 100) //was 100
    {  
      // for (auto& n : velocities)
      // {
      //   averageVel[int(t)] += n;
      // }
      // averageVel[int(t)] = averageVel[int(t)]/velocities.size();
      nextUpdate = 0;
      // std::fill(velocities.begin(), velocities.end(), 0);
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
			//Crashes!!
			if(POSITION_CREATOR == true)
							 {
						std::vector<double> coord_dist_new (3);
						std::vector<double> coord_dist_other (3);

								//Here to make our position CSV, need a last entry for vehicle 0
								for (int z(0); z < 3; z++)
									{
									coord_dist_new.at(z) = quads[i]->getState().p.coeffRef(z,0);  //get our current UAV position.
									coord_dist_other.at(z) = quads[j]->getState().p.coeffRef(z,0);  //get our current UAV position.

									
									}
								std::ofstream pos("Crashes.csv" , std::ios::app );
								pos << i << "," << j << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << "," << t << '\n';
								pos.close();
							 }
          }

        }
    }
  }
  //do something with velocity. Cant really remember
  double finalAverageVel = 0;
  // for (auto& n : averageVel)
  //       finalAverageVel += n;
  //finalAverageVel = finalAverageVel/averageVel.size();
  long int collisionNum = set.size(); //Gets the amount of collisions.
  std::set<std::pair<double,double>>::iterator it = set.begin();
  //this is the file that will get all of the results in a csv. If it does not exist it will be generated.
  std::ofstream file("Heatmap_Test_Standard.csv" , std::ios::app );
	//The variables that are loaded into the csv. 
  file << numVehicles << "," << power << "," << collisionNum << "," << startRadius << "," << maxVelocity << "," << messageOverlap << "," << chance << "," << closeness << "," << collision << "," << MTL << "," << seed << "," << maxAccel << "," << cRadius << "," << cone_range <<'\n';//<< "," << finalAverageVel <<'\n';
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<seconds>(stop - start);

  return 0;
}