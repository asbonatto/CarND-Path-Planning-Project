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

CSys::CSys() {
  // Initializes the Transform with the global coordinate system

  ifstream in_map_(MAP_FILE.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
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
    map_waypoints_normal.push_back(atan2(d_y, d_x));
  }

  //rx_s.set_points(map_waypoints_s, map_waypoints_x);
  //ry_s.set_points(map_waypoints_s, map_waypoints_y);
}

void CSys::check(){
  for (int i = 0; i < map_waypoints_s.size(); i++)
  {
    vector<double> xy = to_cartesian(map_waypoints_s[i], 0);
    double error = distance(map_waypoints_x[i], map_waypoints_y[i], xy[0], xy[1]);
    if (error > 1E-3)
    {
      cout << "ERROR in CSys : distance for point " << i << " : " << error << endl;
    }
  }
}

vector<double> CSys::to_cartesian(double s, double d)
{
  // Interpolating with the splines instead of waypoints
  s = normalize_s(s);

  int prev_wp = get_previous_waypoint_s(s);

  double seg_s = s   - map_waypoints_s[prev_wp];
  double t = get_tangent(s, d);
  double seg_x = map_waypoints_x[prev_wp] + seg_s*cos(t);
  double seg_y = map_waypoints_y[prev_wp] + seg_s*sin(t);

  double n = get_normal(s, d);
  return {seg_x + d * cos(n), seg_y + d * sin(n)};
}

int CSys::get_next_waypoint(double x, double y, double theta){

  int wp = get_closest_waypoint(x,y);

  double map_x = map_waypoints_x[wp];
  double map_y = map_waypoints_y[wp];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*M_PI - angle, angle);

  if(angle > M_PI/4){
    wp++;
    if (wp == map_waypoints_x.size()){
      wp = 0;}
    }
    return wp;
  }

  int CSys::get_closest_waypoint(double x, double y){
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < map_waypoints_x.size(); i++){
      double map_x = map_waypoints_x[i];
      double map_y = map_waypoints_y[i];
      double dist = distance(x,y, map_x, map_y);
      if(dist < closestLen){
        closestLen = dist;
        closestWaypoint = i;
      }
    }
    return closestWaypoint;
  }

  int CSys::get_previous_waypoint_s(double s){
    int prev_wp = -1;

    while(s > map_waypoints_s[prev_wp + 1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
    {prev_wp++;}
    return prev_wp;
  }

  double CSys::get_tangent(double s, double d ){

    int prev_wp = get_previous_waypoint_s(s);
    int wp2 = (prev_wp + 1) % map_waypoints_s.size();

    double angle = atan2( map_waypoints_y[wp2] - map_waypoints_y[prev_wp]
                        , map_waypoints_x[wp2] - map_waypoints_x[prev_wp]);

    return angle;
    }

    double CSys::get_normal(double s, double d){

      return get_tangent(s, d) - M_PI/2;
    }
