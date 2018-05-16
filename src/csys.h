#ifndef CSYS_H
#define CSYS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <vector>
#include <string>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
using namespace tk;

class CSys {
  // implements the mapping of Frenet (road-aligned) to Cartesian coordinates
private:
  // Definition of waypoints
  const string MAP_FILE = "../data/highway_map.csv";

  double MAX_S = 6945.554;
  double LANE_WIDTH = 4.0;
  vector<int> lanes = {0, 1, 2};

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_normal;

  // Using splines to fit the curve so we can access it`s normal/tangent vectors
  //tk::spline rx_s;
  //tk::spline ry_s;
  //tk::spline dx_s;
  //tk::spline dy_s;

  int get_next_waypoint(double x, double y, double theta);
  int get_closest_waypoint(double x, double y);
  int get_previous_waypoint_s(double s);
  void check();

public:
   CSys();
  ~CSys(){};

  // The main methods for the class

  int get_lane(double d){return (int) (d / LANE_WIDTH);}

  // vector<double> to_frenet(double x, double y);
  vector<double> to_cartesian(double s, double d);

  // Helper methods
  double normalize_s(double s)
  {
    return fmod(s, MAX_S);
  }

  double distance(double x1, double y1, double x2, double y2)
  {
  	return l2_norm(x1-x2, y1-y2);
  }

  double l2_norm(double x, double y)
  {
    return sqrt(pow(x, 2) + pow(y, 2));
  }

  double get_tangent(double s, double d);
  double get_normal(double s, double d);

};

#endif
