#include "collision_vo/utils.h"


bool quadratic_formula(double a, double b, double c,
  double& r1, double& r2,
  double& i1, double& i2)
{
  double d=(b*b-4*a*c);
  if (d<0)
  {
    r1 = -b/(2*a);
    r2 = -b/(2*a);
    i1= sqrt(-d)/(2*a);
    i2= -sqrt(-d)/(2*a);
    return false;
  }
  else
  {
    i1=0;
    i2=0;
    r1=(-b+sqrt(d))/(2*a);
    r2=(-b-sqrt(d))/(2*a);
    return true;
  }
}


double check_arcs(double input)
{
  if (input < -1)
    return -1;
  else if (input > 1)
    return 1;
  return input;
}

double get_angle(const Eigen::Vector2d& vect1, const Eigen::Vector2d& vect2)
{
  double atanA = atan2(vect1[0],vect1[1]);
  double atanB = atan2(vect2[0],vect2[1]);
  return atanA - atanB;
}

std::vector<double> linspace(double a, double b, int n)
{
  //https://stackoverflow.com/questions/11734322/matlab-type-arrays-in-c
    std::vector<double> array;
    double step = (b-a) / (n-1);

    while(a <= b) {
        array.push_back(a);
        a += step;
    }
    return array;
}

void switch_points(Eigen::Vector2d& one, Eigen::Vector2d& two)
{
  // A utility function to swap two points
  Eigen::Vector2d temp = one;
  one = two;
  two = temp;
}

int get_orientation(const Eigen::Vector2d& p1,
  const Eigen::Vector2d& p2,
  const Eigen::Vector2d& p3)
{
  // https://www.geeksforgeeks.org/orientation-3-ordered-points/
  // 0==Linear
  // 1==Clockwise
  // Anything else==Counter Clockwise
  double val = (p2[1] - p1[1]) * (p3[0] - p2[0]) -
            (p2[0] - p1[0]) * (p3[1] - p2[1]);

  if (val == 0) return 0;  // colinear

  return (val > 0)? 1: 2; // clock or counterclock wise
}

bool barycentric(const Eigen::Vector2d& A,
    const Eigen::Vector2d& B,
    const Eigen::Vector2d& C,
    const Eigen::Vector2d& P)
{
  // Barycentric Technique: http://blackpawn.com/texts/pointinpoly/
  // For checkking point within a triangle
  Eigen::Vector2d v0 = C - A;
  Eigen::Vector2d v1 = B - A;
  Eigen::Vector2d v2 = P - A;

  double dot00 = v0.dot(v0);
  double dot01 = v0.dot(v1);
  double dot02 = v0.dot(v2);
  double dot11 = v1.dot(v1);
  double dot12 = v1.dot(v2);

  double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
  double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  return (u > 0.0) && (v > 0.0) && (u + v < 1.0);
}

double distance_from_line(const Eigen::Vector2d& A,
  const Eigen::Vector2d& B,
  const Eigen::Vector2d& Pt)
{
  double t1 = (B[1]-A[1])*Pt[0];
  double t2 = (B[0]-A[0])*Pt[1];
  double t3 = B[0]*A[1];
  double t4 = B[1]*A[0];
  double numerator = abs(t1-t2+t3-t4);
  return numerator/sqrt(pow(B[1]-A[1],2)+pow(B[0]-A[0],2));
}

double wrap_angle(double angle, double bound)
{
  if (angle > bound)
    return angle - 2.0 * PI;
  if (angle < bound - 2.0 * PI)
    return angle + 2.0 * PI;
  return angle;

}

double deg_to_rad(double deg)
{
  double PI = 3.14159265359;
  return deg*PI/180;
}
