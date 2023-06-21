#include "collision_vo/min_solve.h"


Polynomial::Polynomial(const double& order,
  const Eigen::VectorXd& coefList,
  const std::vector<Eigen::Vector2d>& polyPoints)
{
  this->order = order;
  coeficients = coefList;
  this->polyPoints = polyPoints;
}

Polynomial::~Polynomial() {}

bool Polynomial::Evaluate(const double* parameters,
                      double* cost,
                      double* gradient) const {
  const double x = parameters[0];
  const double y = parameters[1];
  if(in_poly(x,y))
  {
    cost[0] = get_cost(x,y);
    // std::cout << "Point: " << x << " " << y << "\n";
    // std::cout << "cost1: " << cost[0] << std::endl;
    // Eigen::VectorXd z;
    // Eigen::VectorXd x_in(1);
    // x_in << x;
    // Eigen::VectorXd y_in(1);
    // y_in << y;
    // evaluate_surface(order, x_in, y_in, coeficients, z);
    // std::cout << "cost2: " << z << std::endl;
  }
  else
    return false;
  if (gradient != NULL) {
    gradient[0] = get_gradient_x(x, y);
    gradient[1] = get_gradient_y(x, y);
  }
  return true;
}

bool Polynomial::in_poly(const double& x, const double& y) const
{

  for(int i(0); i<polyPoints.size();i++)
  {
    Eigen::Vector2d one = polyPoints[i];
    Eigen::Vector2d two;
    if(i>=polyPoints.size()-1)
      two = polyPoints[0];
    else
      two = polyPoints[i+1];
    Eigen::Vector2d edge = two - one;
    Eigen::Vector2d test = Eigen::Vector2d{x,y} - one;
    double angle = get_angle(edge, test);


    // std::cout << "Point: " << x << " " << y << "\n";
    // std::cout << "One: " << one << "\n";
    // std::cout << "Two: " << two << "\n";
    // std::cout << "Edge: " << edge << "\n";
    // std::cout << "Test: " << test << "\n";
    // std::cout << "Angle: " << wrap_angle(angle, PI) * 180.0 / PI << "\n\n";
    if(wrap_angle(angle, PI) > 0)
    {
      return false;
    }
  }
  return true;
}

double Polynomial::get_cost(const double& x, const double& y) const
{
  double cost{0};
  double column{0};
  for(int i(0); i<=order; i++)
  {
    for(int j(0); j<=i; j++)
    {
      cost += coeficients[column]*pow(x,i-j)*pow(y,j);
      column++;
    }
  }
  return cost;
}

double Polynomial::get_gradient_x(const double& x, const double& y) const
{
  double cost{0};
  double prevValue{1};
  // std::cout << "Starting X: \n";
  for(int i(0); i<=order-1; i++)
  {
    for(int j(0); j<=i; j++)
    {
      // std::cout << i-j << " " << j << " " << prevValue+j << " " << i-j+1 << std::endl;
      cost += coeficients[prevValue+j]*(i-j+1)*pow(x,i-j)*pow(y,j);
    }
    prevValue = prevValue+i+2;
  }
  return cost;
}

double Polynomial::get_gradient_y(const double& x, const double& y) const
{
  double cost{0};
  double prevValue{2};
  // std::cout << "Starting Y: \n";
  for(int i(0); i<=order-1; i++)
  {
    for(int j(0); j<=i; j++)
    {
      // std::cout << j << " " << i-j << " " << prevValue+j << " " << j+1 << std::endl;
      cost += coeficients[prevValue+j]*(j+1)*pow(x,i-j)*pow(y,j);
    }
    prevValue = prevValue+i+2;
  }
  return cost;
}


void minimize(const double& order,
  const Eigen::VectorXd& Coef,
  const std::vector<Eigen::Vector2d>& polyPoints,
  Eigen::Vector2d& initSolution)
{
  // google::InitGoogleLogging("CVO");
  double parameters[2] = {initSolution[0], initSolution[1]};

  ceres::GradientProblemSolver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::LoggingType::SILENT; 

  ceres::GradientProblemSolver::Summary summary;
  Polynomial* poly = new Polynomial(order, Coef, polyPoints);
  // std::cout << poly->in_poly(-1,0);
  ceres::GradientProblem problem(poly);

  ceres::Solve(options, problem, parameters, &summary);

  initSolution << parameters[0], parameters[1];

  // std::cout << summary.FullReport() << "\n";
  // std::cout << "Initial x: " << init[0] << " y: " << init[1] << "\n";
  // std::cout << "Final   x: " << parameters[0]
  //           << " y: " << parameters[1] << "\n";
}
