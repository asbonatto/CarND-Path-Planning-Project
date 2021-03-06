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
  Updates ego localization parameters from sensor fusion
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

  if (vec_s.size() == 0)
  {
    // Initialization with telemetry
    vec_s = {grd_s, grd_vs, 0};
    vec_d = {grd_d, grd_vd, 0};
    target_lane = current_lane;
    traj_s = JMT(vec_s, MAX_SPEED, 0.8*MAX_ACCEL, 0.8*MAX_JERK);
    traj_d = JMT(vec_d, MAX_SPEED, 0.8*MAX_ACCEL, 0.8*MAX_JERK);
  }
}

void Planner::generate_trajectory(int unused_pts){
  
  path_s.clear();
  path_d.clear();
  path_x.clear();
  path_y.clear();

  int used_pts = NPTS - unused_pts;
  vec_s = traj_s.get_sva(dt*used_pts);
  vec_d = traj_d.get_sva(dt*used_pts);
  
  update_lane_costs();
  
  if (current_lane == target_lane)
  { // Wait for lane change event before choosing another lane
    target_lane = choose_lane();
  }
  
  traj_s.fit(vec_s, {0, lane_speeds[target_lane], 0}, T, true);
  traj_d.fit(vec_d, {csys->get_lane_center(target_lane), 0, 0}, 1.6*T, false);
  
  // Predict new points and blend with previous unused ones
  vector<double> xy;
  double next_s, next_d;
  
  for(int i = 0; i < NPTS; i++)
  {
    double t = dt * (i + 1);
    
    vec_s = traj_s.get_sva(t);
    vec_d = traj_d.get_sva(t);

    next_s = vec_s[0];
    next_d = vec_d[0];
    xy = csys->to_cartesian(next_s, next_d);
    // cout << t << ", " << next_s << ", " << next_d << endl;
    path_s.push_back(next_s);
    path_d.push_back(next_d);
    path_x.push_back(xy[0]);
    path_y.push_back(xy[1]);
  }
  
}

void Planner::scan_road(vector<vector<double>> const &sensor_fusion, bool is_debug){
  /*
  Detects the speeds and distances ahead in each lane
   */
  vehicle_ahead  = {{inf, MAX_SPEED}, {inf, MAX_SPEED}, {inf, MAX_SPEED}};
  vehicle_behind = {{-inf, 0.0}, {-inf, 0}, {-inf, 0}};
  
  for (int i = 0; i < sensor_fusion.size(); i++)
  {
      // id, x, y, vx, vy, s, d
      int lane = csys->get_lane(sensor_fusion[i][6]);
      if (lane > -1)
      { // In real case scenario, I would have to deal with oncoming traffic
        double x = sensor_fusion[i][1];
        double y = sensor_fusion[i][2];
        double distance = sensor_fusion[i][5] - grd_s;
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double theta = csys->get_tangent(x, y);
        double vs = max(vx*cos(theta) + vy*sin(theta), 0.001);

        if (distance >= 0 and distance <= vehicle_ahead[lane][0])
        { // Mapping vehicles ahead
          vehicle_ahead[lane][0] = distance; 
          vehicle_ahead[lane][1] = vs;
        }
        if(distance < 0 and distance>=vehicle_behind[lane][0])
        { // Mapping vehicles behind
          vehicle_behind[lane][0] = distance; 
          vehicle_behind[lane][1] = vs;
        }
      }
  }
}

double Planner::time_to_collision(int lane){
  double rel_v = max(vehicle_behind[lane][1] - grd_vs, 0.0);
  
  double tc = inf;
  if (rel_v > 1E-3){
    return - vehicle_behind[lane][0]/rel_v;
  }
  
  return tc;

}

vector<int> Planner::get_available_lanes(int lane){
  
  vector<int> next_lanes;
  for (int l = min(0, lane - 1); l < max(3, lane + 1); l++)
  {
    next_lanes.push_back(l);
  }
  return next_lanes;
}

double Planner::get_lane_speed(int lane){
  // Finds the highest speed that satisfies break safety constraint

  double distance = vehicle_ahead[lane][0]; 
  double tgt_speed = min(MAX_SPEED, sqrt(2*MAX_ACCEL*distance));

  return tgt_speed;
}

void Planner::update_lane_costs(){
  double min_cost = inf;

  for (int l = 0; l < 3; l++)
  {
    lane_speeds[l] = get_lane_speed(l);
    lane_costs[l] = 1 - logit(lane_speeds[l]/MAX_SPEED); // Efficiency cost
    lane_costs[l] += 0.005*pow(l - current_lane, 2); // Lateral movement cost
    lane_costs[l] += 1*fabs(l - current_lane)*pow(vehicle_ahead[l][0], -2); // Collision prevention cost
    lane_costs[l] += 1*fabs(l - current_lane)*pow(vehicle_behind[l][0], -2); // Collision prevention cost
    
  }
}

int Planner::choose_lane(){
  
  double min_cost = inf;
  int next_lane = current_lane;
  
  for (int l = max(0, current_lane - 1); l < min(3, current_lane + 2); l++)
  { /*NOTE :
      We are introducing left-lane bias to the code when we start
    searching from the left to the right and using strict inequality
    for choosing the target lane
    */
  
    if (lane_costs[l] < min_cost and time_to_collision(l) > 2.0*fabs(l - current_lane))
    {
      min_cost = lane_costs[l];
      next_lane = l;
    }
  }
  // std::distance(lane_costs.begin(), std::min_element(lane_costs.begin(), lane_costs.end()))
  return next_lane;
}