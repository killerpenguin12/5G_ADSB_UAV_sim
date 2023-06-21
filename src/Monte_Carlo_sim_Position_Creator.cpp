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
#include <random>
//#include "uatModel.h"
//#include "vehicleMSO.h"
#include <sys/resource.h>

#include "waypoint_generator.h"
using namespace std::chrono;

int main(int argc, char** argv)
{
  bool use_random_seed;
  double t(0), tf, dt, cRadius, startRadius,maxVelocity,cone_range;
  int numVehicles;
  float power;
  double maxAccel;
  auto start = high_resolution_clock::now();
  numVehicles = std::stoi(argv[1]);
  startRadius = std::stoi(argv[2]);
  power = std::stof(argv[3]);
  maxVelocity = std::stof(argv[4]);
  cRadius = std::stod(argv[5]);
  cone_range = std::stod(argv[6]);

  

  // std::string uavSimPath = ros::package::getPath("uav_sim");
  common::get_yaml_node("use_random_seed", "../params/sim.yaml", use_random_seed);
  common::get_yaml_node("tf", "../params/sim.yaml", tf);
  common::get_yaml_node("dt", "../params/sim.yaml", dt);
  //common::get_yaml_node("number_vehicles", "../params/sim.yaml", numVehicles);
  //common::get_yaml_node("start_radius", "../params/sim.yaml", startRadius);
  //common::get_yaml_node("max_velocity", "../params/sim.yaml", maxVelocity);

  common::get_yaml_node("collision_radius", "../params/cvo.yaml", cRadius); //what size counts as a collision.
  //common::get_yaml_node("effective_radiated_power", "../params/sim.yaml", power); //radiated power for each uav, options are 0.01,0.05,0.1,1.0,10.0,20.0
  //numVehicles = 3;

  //int seed = 1104279789;
  //int seed = std::chrono::system_clock::now().time_since_epoch().count();
  int seed = 1088184109;
  //int seed = -418989735;
  std::mt19937_64 generator (seed);
  std::uniform_real_distribution<double> dis(0.0, 1.0); //for our regular random numbers
  std::uniform_int_distribution<int> uni{0, 3601}; //for our initial frame.
  // /std::cout << "Power: " << power << std::endl;
  if (use_random_seed){
    //seed = std::chrono::system_clock::now().time_since_epoch().count();
    //seed = 0;
    //seed = -949817955;
    // /std::srand(seed);
    //std::cout << "random" << std::endl;
    //std::mt19937_64 generator (seed);
    //std::uniform_real_distribution<double> dis(0.0, 1.0); //for our regular random numbers
    //std::uniform_int_distribution<int> uni{0, 3601}; //for our initial frame.
    //std::srand(M_PI);
  }
  //Our power libraries, they dont change so it works well for them to be included this way. Makes compilation long though. 
  int MAX_dist = 0;
  float const *ERP_power;
  //choose the proper power list:
  //ERP_power will contain a pointer to the correct list of values. 
  if(power < 0.05)
  {
    
    float ERP_power_hundreth[3446];
    
    std::fstream myfile("../include/WattValues/hundreth_watt.txt", std::ios_base::in);
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
    float ERP_power = {0};
    std::cout << "Power dict problem! Does not match our areas." << std::endl; //our error
  }
  // Create progress bar
  //common::ProgressBar prog_bar;
  //prog_bar.init(tf/dt,40);

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
    //memory leak here
    quadrotor::Quadrotor* quad = new quadrotor::Quadrotor("../params/quadrotor.yaml",
        name,
        waypoints,
        env,
        use_random_seed,
        i);
    
    quads[i] = quad;
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
        dt*100,cone_range,maxAccel);
    quadCVOManagers[i] = quadCVOManager;

     
    env.initVehicle(quad->getState().p, quad->getState().v, quad->id_);
    

  }
  for(int i = 0; i < numVehicles; i++)
  {
     //random between 0 and 3601 for our frame 
    int random_num = uni(generator);//rand() % 3601;
    //making us start at zero time for our frame
    //quads[i]->initMSO(random_num,i);  
    quads[i]->initMSO(0,i);  
  }
  
  vehicle::Stated allXC[numVehicles];

  //Our counters for collisions:
  long int chance = 0;
  long int MTL = 0;
  long int messageOverlap = 0;
  long int collision = 0;
  long int closeness = 0;
  

  // Main simulation loop
  
  int nextUpdate = 0;
  std::set<std::pair<double,double>> set;
  std::map<std::pair<double,double>, double> map;
  
  // CPyInstance pyInstance;
  std::vector<std::vector<int>> visibilityList; //std::vector<int> (0,0);
  for(int i = 0; i < numVehicles; i++)
  {
    std::vector<int> vec(0,0);
    visibilityList.push_back(vec);

  }
  //visibilityList.push_back(vec);
  std::vector<int> collisionList = std::vector<int> (0,0);
  int num_mso_total = 0;
  int check = 0;
  // int count_coulton = 0;
  // long int count_transmission = 0;
  // long int count_decode_dict = 0;
  // long int successful_count = 0;
  while (t <= tf)
  {
    
    // t += dt;
    // //prog_bar.print(int(t)/dt);
    nextUpdate += 1;

    // Run each vehicle
   
    int count = 0;
    for(int i(0); i<numVehicles; i++)
    {
      int count_Coulton = 0;
      //std::vector<int> vec(0,0);
      quadCVOManagers[i]->propagate(t);
      
        if(nextUpdate <= 1) // tried = 99
        { //once every second 
        std::vector<double> coord_dist_1 (3);
        //Here to make our position CSV, need a last entry for vehicle 0
        for (int j(0); j < 3; j++)
            {
            coord_dist_1.at(j) = quads[i]->getState().p.coeffRef(j,0);  //get our current UAV position.
            
            }
          std::ofstream pos("Positions.csv" , std::ios::app );
          pos << i << "," << coord_dist_1[0] << "," << coord_dist_1[1] << 1 << '\n';
          pos.close();
        if(count == numVehicles - 1)
        {
            std::vector<std::vector<int>> sec; 
            std::vector<int> distance;
            std::vector<int> mso_list;
            std::vector<double> coord_dist (3);
            //get our current position for this quadrotor:
            for (int j(0); j < 3; j++)
            {
            coord_dist.at(j) = quads[i]->getState().p.coeffRef(j,0);  //get our current UAV position.
            
            }
            //This is a vector of a pair of vectors. the inside vectors will contain 
            //distances and vehicle numbers respectively. The index of the containing vector
            //is the recieving vehicle number.
            std::vector<std::vector<std::pair<int,int>>> distanceDict;
            std::vector<float> recieved_powerDict;
            std::vector<std::vector<bool>> successful_decodeDict;
            std::vector<std::vector<bool>> successful_decodeDict_test; 
            
            //std::cout << "3" << std::endl;
            //std::vector<std::pair<int,std::vector<int>>> sorted_list;
            std::vector<std::pair<int,int>> sorted_list;
            for(int j = 0; j < numVehicles; j++)
            {
                quads[j]->updateMSO();
                //std::cout << "Time: " << t << std::endl;
                int mso_saved = quads[j]->full_mso_range();
                num_mso_total += 1;

                mso_list.push_back(mso_saved);
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
                //MTLcounter.push_back(meets_minimum_trigger_level);
                //count_transmission += 1;
                MTLcounter[(k%numVehicles)] = meets_minimum_trigger_level;
                //distances.push_back(std::make_pair(distance,(k%numVehicles)));
                recieved_powerDict.push_back(recieved_power);
                distances.push_back(std::make_pair(distance,(j)));
                //std::cout << "Distance " << j << ":" << distance << ", Transmission ID: " << k % numVehicles << std::endl;
                //count_Coulton +=1;
                
                }
              // /std::cout << "Count Coulton: " << count_Coulton << std::endl;
              distanceDict.push_back(distances);
              successful_decodeDict.push_back(MTLcounter);
              successful_decodeDict_test.push_back(MTLcounter);
              sec.push_back(mso_list);
              quads[j]->increaseFrame();
              sorted_list.push_back(std::make_pair(mso_saved,j));
              
             
            }
            
              for(int k = 0; k < numVehicles; k++)
              {
                successful_decodeDict[k][k] = 0;
                //Set our own vehicles so they cannot transmit to themselves
              }
            long int failure_count = 0;
            for(int k = 0; k < successful_decodeDict.size(); k++)
            {
              for(int j = 0; j < successful_decodeDict[k].size();j++)
              {
                if(successful_decodeDict[k][j] == 0)
                {
                  
                  failure_count += 1; //whose messages have failed. 
                  //so vehicle k cannot see vehicle j if it is in this list.
                }
              }
            }
            //std::cout << "MTL/chance failure count: " << failure_count << std::endl;
            for(int k = 0; k < numVehicles; k++)
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
                    // std::cout << "Closeness" << std::endl;
                    // std::cout << "k mso: " << k_mso << " , g_mso: " << g_mso << std::endl;
                    // std::cout <<"First index: " << k << " , Second index: " << g << std::endl;
                    // for(int x = 0; x < successful_decodeDict.size(); x++)
                    // {
                    //   for(int y = 0; y < successful_decodeDict[x].size(); y++)
                    //     std::cout << successful_decodeDict[x][y] << ", ";
                    //   std::cout << std::endl;
                    // }
                    
                    std::vector<double> coord_dist_new (3);
                    std::vector<double> coord_dist_other (3);

                    //Here to make our position CSV, need a last entry for vehicle 0
                    for (int z(0); z < 3; z++)
                        {
                        coord_dist_new.at(z) = quads[k]->getState().p.coeffRef(z,0);  //get our current UAV position.
                        coord_dist_other.at(z) = quads[g]->getState().p.coeffRef(z,0);  //get our current UAV position.

                        
                        }
                    std::ofstream pos("Collisions.csv" , std::ios::app );
                    pos << k << "," << g << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << '\n';
                    pos.close();
                    }
                  }
              }
            }
            failure_count = 0;
            for(int k = 0; k < successful_decodeDict.size(); k++)
            {
              for(int j = 0; j < successful_decodeDict[k].size();j++)
              {
                if(successful_decodeDict[k][j] == 0)
                {
                  
                  failure_count += 1; //whose messages have failed. 
                  //so vehicle k cannot see vehicle j if it is in this list.
                }
              }
            }
            //std::cout << "MTL/chance/closeness failure count: " << failure_count << std::endl;
              //Overlap Calcualtion section
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
                      //if(sorted_list[k].second == g)
                      //{continue;} //dont want transmitter and reciever to be the same.
                      if(successful_decodeDict[sorted_list[k].second][g] == true)
                      {
                        successful_decodeDict[sorted_list[k].second][g] = false;
                        messageOverlap += 1;
                        // std::cout << "Overlap" << std::endl;
                        // std::cout << "Previous MSO: " << previousMSO << std::endl;
                        // std::cout <<"First index: " << sorted_list[k].second << " , Second index: " << g << std::endl;
                        // for(int x = 0; x < successful_decodeDict.size(); x++)
                        // {
                        //   for(int y = 0; y < successful_decodeDict[x].size(); y++)
                        //     std::cout << successful_decodeDict[x][y] << ", ";
                        //   std::cout << std::endl;
                        // }
                        
                        std::vector<double> coord_dist_new (3);
                        std::vector<double> coord_dist_other (3);

                    //Here to make our position CSV, need a last entry for vehicle 0
                    for (int z(0); z < 3; z++)
                        {
                        coord_dist_new.at(z) = quads[k]->getState().p.coeffRef(z,0);  //get our current UAV position.
                        coord_dist_other.at(z) = quads[g]->getState().p.coeffRef(z,0);  //get our current UAV position.

                        
                        }
                    std::ofstream pos("Collisions.csv" , std::ios::app );
                    pos << k << "," << g << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << '\n';
                    pos.close();
                      }
                    }
              }
              else
                num_consecutive = 0;
              previousMSO = sorted_list[k].first;
            }
            // failure_count = 0;
            // for(int k = 0; k < successful_decodeDict.size(); k++)
            // {
            //   for(int j = 0; j < successful_decodeDict[k].size();j++)
            //   {
            //     if(successful_decodeDict[k][j] == 0)
            //     {
                  
            //       failure_count += 1; //whose messages have failed. 
            //       //so vehicle k cannot see vehicle j if it is in this list.
            //     }
            //   }
            // }
            //std::cout << "MTL/chance/closeness/overlap failure count: " << failure_count << std::endl;
            // '''
            // IMPORTANT END SECTION TO COMMENT FOR MESSAGE ENHANCED MODEL
            // TODO missing consecutive message case of low power first message high power second message
            // Now we look at MSO collisions. First we record the collisions, then we deal with their implications
            std::vector<std::pair<int,std::vector<int>>> collisionObjects; //to collect all our collisions in. Put in our mso and vehicle number. 
            std::vector<int> vehiclesInvolved;
            std::vector<std::vector<int>> totalVehiclesInvolved;
            std::vector<std::pair<int,int>> transmissions;
            //Get our Transmitting Vehicles!
            int collision_count = 0;
            for(int k =752; k < 3952;k++) //go through all msos
            {
              //need to count occurances in our vector of vector ints:
              int count = 0;
              for(int ele : sec[numVehicles-1])
              {
                if (ele  == k)
                  count += 1; //Counting occurances of that particular MSO as given by k
              }
              if(count > 1) //if its more than one then we have a collision
              {
                vehiclesInvolved.clear();
                for(int g = 0; g < numVehicles; g++) //check all our vehicles
                { 
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
                int mso_to_check = quads[g]->getMSO();
                //std::cout << "mso_to_check: " << mso_to_check << std::endl;
                //that is not part of the collision (and not switching modes)
                if (mso_to_check < (mso - 9) || (mso_to_check > (mso + 8)))
                {
                  transmissions.clear(); //make it clean
                  // CollisionObjects = if we had a collision, what vehicles were involved. Second is all the vehicle numbers
                  // compare to each vehicle that was part of the collision (add them to a list)
                  //std::cout << std::endl;
                  //std::cout << "size: " << collisionObjects[k].second.size() << std::endl;
                  for(int h = 0; h < collisionObjects[k].second.size(); h++) //vehicles involved
                  { 
                        transmissions.push_back(std::make_pair(distanceDict[collisionObjects[k].second[h]][g].first,collisionObjects[k].second[h]));
                  }
                  std::sort(transmissions.begin(),transmissions.end());
                  float recieved_power = ERP_power[transmissions[0].first];
                  //If the transmission from the closest vehicle meets the MTL
                  if(transmissions.size() > 1)
                  if(ERP_power[transmissions[0].first] >= -93.0) //93 is the minimum_trigger_level
                  {
                    //  IMPORTANT
                    //  The "3000" is the difference between this file and main_sim_collision_enhanced.py. Change the
                    //  3000 to p.power_difference_decibel to get collision enhanced model.
                    //  If the power difference between the closest and second-closest vehicle is greater than or
                    //  equal to the necessary power difference
                      if(ERP_power[transmissions[0].first] - ERP_power[transmissions[1].first] >= 3000) //or 4, or 3000
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
                              collision_count += 1;
                            }
                          }
                        }

                      } 
                      else if (ERP_power[transmissions[1].first] < -93.0) 
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
                              collision_count += 1;
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
                              collision_count += 1;
                              }
                            // std::cout << "Collisions" << std::endl;
                            // std::cout << "Time: " << t << std::endl; 
                            // std::cout << "MSO to check: " << mso_to_check << std::endl;
                            // std::cout << "MSO: " << mso << std::endl;
                            // std::cout <<"First index: " << indices[y] << " , Second index: " << g << std::endl;
                            // std::cout << "Collision Count: " << collision << std::endl;
                            // // std::cout << "Transmission " << y << std::endl;
                            // // for(int x = 0; x < successful_decodeDict.size(); x++)
                            // // {
                            // //   for(int p = 0; p < successful_decodeDict[x].size(); p++)
                            // //     std::cout << successful_decodeDict[x][p] << ", ";
                            // //   std::cout << std::endl;
                            // // }
                            // std::cout << "#######################################################" << std::endl;
                            std::vector<double> coord_dist_new (3);
                            std::vector<double> coord_dist_other (3);

                            // Here to make our position CSV, need a last entry for vehicle 0
                            for (int z(0); z < 3; z++)
                            {
                              coord_dist_new.at(z) = quads[k]->getState().p.coeffRef(z,0);  //get our current UAV position.
                              coord_dist_other.at(z) = quads[g]->getState().p.coeffRef(z,0);  //get our current UAV position.

                          
                            }
                            std::ofstream pos("Collisions.csv" , std::ios::app );
                            pos << k << "," << g << "," << coord_dist_new[0] << "," << coord_dist_new[1] << "," << coord_dist_other[0] << "," << coord_dist_other[1] << '\n';
                            pos.close();
                            }
                            
                         } 
                            

                           

                         }
                      //}
                  
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
                                collision_count += 1;
                              }
                            }
                            
                         } 
                        // if(successful_decodeDict[transmissions[y].second][g] == true)
                        //   {
                        //     successful_decodeDict[transmissions[y].second][g] == false;
                        //     collision += 1;
                        //     std::cout << "Does this activate? " << std::endl;
                        //   } 

                  }
                }
              }
            }
            failure_count = 0;
            for(int k = 0; k < successful_decodeDict.size(); k++)
            {
              for(int j = 0; j < successful_decodeDict[k].size();j++)
              {
                if(successful_decodeDict[k][j] == 0)
                {
                  
                  failure_count += 1; //whose messages have failed. 
                  //so vehicle k cannot see vehicle j if it is in this list.
                }
              }
            }
            // std::cout << "All failure count: " << failure_count << std::endl;
            // std::cout <<"Collision Count: " << collision_count << std::endl;
            // std::cout << "Transmissions this second: " << numVehicles*numVehicles <<std::endl;
            // std::cout << "Overlap: " << messageOverlap << std::endl;
            for(int k = 0; k < successful_decodeDict.size(); k++)
            {
              for(int j = 0; j < successful_decodeDict[k].size();j++)
              {
                //successful_count += 1;
                if(successful_decodeDict[k][j] == 0)
                {
                  
                  visibilityList[k].push_back(j); //whose messages have failed. 
                  //so vehicle k cannot see vehicle j if it is in this list.
                }
              }
            }
            //std::cout << "successful Decode count: " << count_decode_dict << std::endl;
            count += 1;

            //  std::ofstream file("Test_every_second.csv" , std::ios::app );
            // //file << "numVehicles" << "," << "power" << "," << "time" << "," << "startRadius" << "," << "maxVelocity" << "," << "messageOverlap" << "," << "chance" << "," << "closeness" << "," << "collision" << "," << "MTL" << "," << "seed" << '\n';
            // file << numVehicles << "," << power << "," << t << "," << startRadius << "," << maxVelocity << "," << messageOverlap << "," << chance << "," << closeness << "," << collision << "," << MTL << "," << seed << '\n';
            // // auto stop = high_resolution_clock::now();
          }
          vehicle::Stated xc = quadWPManagers[i]->updateWaypointManager(quads[i]->getState());
          quadCVOManagers[i]->get_best_vel(quads[i]->getState(), t, xc, visibilityList[i]);
          allXC[i] = xc;
          count += 1;
          visibilityList[i].clear();
        }
      
      quadrotor::uVector u = quadControllers[i]->computeControl(quads[i]->getState(), t, allXC[i]); //returning all the throttle,yaw and 
      quads[i]->run(t, env, u);
    }
    t += dt;
    // //prog_bar.print(int(t)/dt);
    //nextUpdate += 1;
    if(nextUpdate >= 100) //was 100
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
  std::set<std::pair<double,double>>::iterator it = set.begin();
  //std::cout << "Succesful decode size: " << successful_count << std::endl;
  std::ofstream file("Final_Results_test_100.csv" , std::ios::app );
  //file << "numVehicles" << "," << "power" << "," << "collisionNum" << "," << "startRadius" << "," << "maxVelocity" << "," << "messageOverlap" << "," << "chance" << "," << "closeness" << "," << "collision" << "," << "MTL" << "," << "seed" << '\n';
  file << numVehicles << "," << power << "," << collisionNum << "," << startRadius << "," << maxVelocity << "," << messageOverlap << "," << chance << "," << closeness << "," << collision << "," << MTL << "," << seed << '\n';
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<seconds>(stop - start);
  //std::cout << "Collisions" << collision << std::endl;
  //std::cout << "successful_decode_dict: " << count_decode_dict << std::endl;
  //float percentage = collision/count_transmission;
  // std::cout << "Collisions percentage: " << percentage << std::endl;
  // std::cout << "Transmissions count: " << count_transmission << std::endl;
  //std::cout << "Crashes: " << collisionNum << std::endl;
  //std::cout << duration.count() << std::endl;

  return 0;
}
