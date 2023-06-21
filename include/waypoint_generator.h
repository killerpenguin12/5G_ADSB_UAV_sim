#pragma once
#include <chrono>
#include <random>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <math.h>

class WaypointGenerator
{

public:
  WaypointGenerator(double startRadius, double numVehicles, double vehicleRadius, int seed)
  {
    startRadius_ = startRadius; // now the length of side of square
    numVehicles_ = numVehicles;
    vehicleRadius_ = vehicleRadius;

    //int seed = std::chrono::system_clock::now().time_since_epoch().count();
    //rng_ = std::default_random_engine(seed);
    srand(seed);
    //srand(5);
  };
  ~WaypointGenerator() {};

  std::vector<Eigen::Vector2d> get_waypoints()
  {

    // return std::vector<Eigen::Vector2d>{Eigen::Vector2d(100,0), Eigen::Vector2d(-100,0),
    //     Eigen::Vector2d(-100,0), Eigen::Vector2d(100,0),
    //     Eigen::Vector2d(0,100), Eigen::Vector2d(0,-100),
    //     Eigen::Vector2d(0,-100), Eigen::Vector2d(0,100)};

    std::vector<Eigen::Vector2d> allPositions;
    std::vector<Eigen::Vector2d> allWaypoints;

    Eigen::Vector2d point = get_rand_position();
    allPositions.push_back(point);
    allWaypoints.push_back(point);
    allWaypoints.push_back(-point);
    // allWaypoints.push_back(-point/point.norm()*100000);


    for(int i(0); i<numVehicles_-1;i++)
    {
      bool foundOther = true;
      while(foundOther)
      {
        point = get_rand_position();
        for(auto pos : allPositions)
        {
          if( (pos-point).norm() < vehicleRadius_*5.0 )
          {
            // std::cout << (pos-point).norm() << std::endl;
            foundOther = true;
            break;
          }

          foundOther = false;
        }
      }
      allPositions.push_back(point);
      allWaypoints.push_back(point);
      allWaypoints.push_back(-point);
    }
    return allWaypoints;

  };

  std::vector<Eigen::Vector2d> get_even_waypoints()
  {
    
    // std::vector<Eigen::Vector2d> allPositions;
    std::vector<Eigen::Vector2d> allWaypoints;
    Eigen::Vector2d point1;
    Eigen::Vector2d point2;
    for(int i(0); i<numVehicles_;i++)
    {
      //Random start position and goes accross the square
      //#####################################################################
          //  point1 = get_start_position_even(i);
          // Eigen::Vector2d temp = get_position(point1);
          //  point2 = get_end_position_even(i);
           //point2 = get_straight_position(-point1[1],-point1[0]);//get_rand_position();
          //  allWaypoints.push_back(point1);
          //  allWaypoints.push_back(point2);
      //##########################################################################
      //Random start and end positions
      //#######################################################################
          //std::cout << "Start Position: " << std::endl;
          point1 = get_rand_position();
          // //std::cout << "End Position: " << std::endl;
          point2 = get_rand_position();
          allWaypoints.push_back(point1);
          allWaypoints.push_back(point2);
      // //##########################################################################
      // Vehicle Test
      //#######################################################################
        // if(i%2 == 0)
        //   {
        //    point1 = get_straight_position(100,0);
        //    point2 = get_straight_position(0, 0);
        //   } 
        // else
        //   {
        //    point1 = get_straight_position(-100,0);
        //    point2 = get_straight_position(0, 0);
        //   }
        // allWaypoints.push_back(point1);
        // allWaypoints.push_back(point2);
      // //##########################################################################
      // Vehicle Test 2
      //#######################################################################
        // if(i%2 == 0)
        //   {
        //    point1 = get_straight_position(0,100);
        //    point2 = get_straight_position(0, 0);
        //   } 
        // else
        //   {
        //    point1 = get_straight_position(0,-100);
        //    point2 = get_straight_position(0, 0);
        //   }
        // allWaypoints.push_back(point1);
        // allWaypoints.push_back(point2);
      //##########################################################################
      //only works with 3 vehicles
      //#############################################################################
      // {
        //start_position = (i*10,area_radius,beginning_altitude)
        //end_position = (-i*10,area_radius,beginning_altitude)
      //     point1 = get_straight_position(startRadius_,2*i);
      //     point2 = get_straight_position(-startRadius_,2*i);
      // //point = get_position(i);
      // //allPositions.push_back(point1);
      // allWaypoints.push_back(point1);
      // allWaypoints.push_back(point2);
      // }
    } 
    return allWaypoints;
  };  

private:

  double startRadius_; //now length of side of square
  double numVehicles_;
  double vehicleRadius_;
  double PI{3.14159265359};

  Eigen::Vector2d get_position(int position)
  {
    double theta;
    if (position >= 5)
    {
      theta = 2*PI*(position+1)/numVehicles_ - PI/8;
    } else {
      theta = 2*PI*(position+1)/numVehicles_;
    }
    double x = startRadius_*cos(theta);
    double y = startRadius_*sin(theta);
    return Eigen::Vector2d(x,y);
  }
  Eigen::Vector2d get_start_position_even(int i)
  {
  double x = startRadius_*cos(i*(4/PI));//) + startRadius_;
  double y = startRadius_*sin(i*(4/PI));// + startRadius_;
  
  return Eigen::Vector2d(x,y);
  };

  Eigen::Vector2d get_end_position_even(int i)
  {
  double x = startRadius_*cos(i*(4/PI) + PI);// + startRadius_;
  double y = startRadius_*sin(i*(4/PI) + PI);// + startRadius_;
  //std::cout << "Start_radius: "  << startRadius_ << std::endl;
  
  return Eigen::Vector2d(x,y);
  };

  Eigen::Vector2d get_rand_position()
  {
    //Just gets a random point on a circle.
    double min = -PI;
    double max = PI;
    double randTheta = (((double) rand() / RAND_MAX) * (max-min)) + min; //some decimal *
    double randNumber = ((double) rand()/RAND_MAX); //used to get a random number between 0 and 1
    //std::cout << "Random Number: " << randNumber << std::endl;
    //std::cout << "Theta: " << randTheta << std::endl;
    // double x = (startRadius_)*cos(randTheta);
    // double y = (startRadius_)*sin(randTheta);
    double x = (startRadius_*randNumber)*cos(randTheta);
    double y = (startRadius_*randNumber)*sin(randTheta);
    return Eigen::Vector2d(x,y);
  };

  Eigen::Vector2d get_rand_start_position()
  {
    //Starting here on a square.
    //double x = 0;
    //double y = 0; 
    //double x = static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(startRadius_)));
    //double y = static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(startRadius_)));
    double min = -PI;
    double max = PI;
    double randTheta = (((double) rand() / RAND_MAX) * (max-min)) + min; //some decimal * 
    //std::cout << "Theta: " << randTheta << std::endl;
    double x = startRadius_;//*cos(randTheta);
    double y = startRadius_;//*sin(randTheta);
    return Eigen::Vector2d(x,y);
  };

  Eigen::Vector2d get_straight_position(int xPosition, int yPosition)
  {
    double xWeight = 1;
    double yWeight = 1;
    return Eigen::Vector2d(xPosition*xWeight,yPosition*yWeight);
  };

};
