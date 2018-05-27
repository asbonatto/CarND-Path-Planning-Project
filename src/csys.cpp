#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <vector>
#include <string>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "csys.h"
using namespace std;
using namespace tk;

CSys::CSys()
{
  // Initializes the Transform with the global coordinate system

  ifstream in_map_(MAP_FILE.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
  }
  
  // Fixing discontinuity at the end of the loop before fitting the spline
  double x_i = map_waypoints_x.front();
  double y_i = map_waypoints_y.front();
  double s_i = map_waypoints_s.front() + MAX_S;
  
  double x_f = map_waypoints_x.back();
  double y_f = map_waypoints_y.back();
  double s_f = map_waypoints_s.back() - MAX_S;

  map_waypoints_x.insert(map_waypoints_x.begin(), x_f);
  map_waypoints_y.insert(map_waypoints_y.begin(), y_f);
  map_waypoints_s.insert(map_waypoints_s.begin(), s_f);

  map_waypoints_x.push_back(x_i);
  map_waypoints_y.push_back(y_i);
  map_waypoints_s.push_back(s_i);

  rx_s.set_points(map_waypoints_s, map_waypoints_x);
  ry_s.set_points(map_waypoints_s, map_waypoints_y);
  
}

vector<double> CSys::to_cartesian(double s, double d)
{
  // Interpolating with the splines instead of waypoints
  s = normalize_s(s);
  double n = get_normal(s, d);
  return {rx_s(s) + d * cos(n), ry_s(s) + d * sin(n)};
}

double CSys::get_tangent(double s, double d)
{  
  return get_normal(s, d) + M_PI/2;
}

double CSys::get_normal(double s, double d)
{
      return atan2(-rx_s.deriv(1, s), ry_s.deriv(1, s));
}
