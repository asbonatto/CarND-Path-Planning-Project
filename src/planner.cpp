#include "planner.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <limits>
#include "spline.h"
#include "csys.h"

Planner::Planner(CSys* road_csys) : csys(road_csys){}

void Planner::update_telemetry(double x, double y, double s, double d, double yaw, double speed, int unused_pts){
  /*
  */
  yaw = deg2rad(yaw);
  speed = mph2mps(speed);

  grd_s = s;
  grd_d = d;

  current_lane = csys->get_lane(d);

  double vx = speed*cos(yaw);
  double vy = speed*sin(yaw);

  double theta = csys->get_tangent(x, y);
  grd_vs = vx * cos(theta) + vy * sin(theta);

  theta = csys->get_normal(x, y);
  grd_vd = vx * cos(theta) + vy * sin(theta);

  if (last_s.size() == 0)
  {
    // Initialization with telemtry
    last_s = {grd_s, grd_vs, 0};
    last_d = {grd_d, grd_vd, 0};
    target_lane = current_lane;
  }
  initial_s = last_s;
  initial_d = last_d;
}

void Planner::generate_trajectory(int unused_pts){

  if(unused_pts > 0){
    path_s.erase(path_s.begin(), path_s.begin() + NPTS - unused_pts);
    path_d.erase(path_d.begin(), path_d.begin() + NPTS - unused_pts);
    path_x.erase(path_x.begin(), path_x.begin() + NPTS - unused_pts);
    path_y.erase(path_y.begin(), path_y.begin() + NPTS - unused_pts);
  }

  vector<double> xy;
  double next_s, next_d;
  JMT traj_s = JMT();
  traj_s.fit(initial_s, {0, MAX_SPEED, 0}, 3.0, true);
  // traj_s.s = {initial_s[0], MAX_SPEED, 0};
  // traj_s.derivatives();
  // traj_s.polyprint(traj_s.v);

  for(int i = unused_pts; i < NPTS; i++)
  {// Lane keeper
    double t = dt * (i + 1 - unused_pts);
    last_s = traj_s.get_sva(t);
    next_s = last_s[0];
    next_d = grd_d;
    xy = csys->to_cartesian(next_s, next_d);

    path_s.push_back(next_s);
    path_d.push_back(next_d);
    path_x.push_back(xy[0]);
    path_y.push_back(xy[1]);
  }
  cout << "s, v, a | ";
  cout << last_s[0] << ", " << last_s[1] << ", " << last_s[2] << endl;

}

void Planner::scan_road(vector<vector<double>> const &sensor_fusion, bool is_debug){
  /*
  Detects the speeds and distances ahead in each lane
   */
  lane_speeds = {MAX_SPEED, MAX_SPEED, MAX_SPEED};
  lane_distances = {inf, inf, inf};

  for (int i = 0; i < sensor_fusion.size(); i++)
  {
      // id, x, y, vx, vy, s, d
      int lane = csys->get_lane(sensor_fusion[i][6]);
      if (lane > -1)
      { // In real case scenario, I would have to deal with oncoming traffic
        double x = sensor_fusion[i][1];
        double y = sensor_fusion[i][2];
        double distance = sensor_fusion[i][5] - grd_s;
        if (distance >= 0 and distance <= lane_distances[lane])
        { // Closest vehicle ahead so far
          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];
          double theta = csys->get_tangent(x, y);
          double vs = vx*cos(theta) + vy*sin(theta);
          lane_speeds[lane] = vs;
          lane_distances[lane] = distance;
        }
      }
  }
  if (is_debug){
    cout << "Car @ lane # " << current_lane << endl;
    for (int i = 0; i < lane_distances.size(); i++){
      cout << "Lane # " << i << endl;
      cout << "    Vehicle ahead s, v : " << lane_distances[i] << ", " << lane_speeds[i] << endl;
    }
  }
}
