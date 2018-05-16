#include "road.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <limits>

Road::~Road() {}

Road::Road(double speed_limit, int nlanes, double lane_width) {
    /**
    * Initializes Road with scene parameters
    */

    this->nlanes = nlanes;
    this->speed_limit = speed_limit;
    this->lane_width = lane_width;

    double inf = std::numeric_limits<double>::infinity();
    // Initialize with allowable speed
    this->lane_front_speed = {speed_limit, speed_limit, speed_limit};
    this->lane_front_distance = {inf, inf, inf};

    this->lane_back_speed = {0., 0., 0.};
    this->lane_back_distance = {-inf, -inf, -inf};
}

int Road::get_lane(double d) {
    /**
    * Returns the lane associated with coordinate d
    */
    int lane = -1;
    if (d >= 0){
      lane = (int) (d / this->lane_width);
    }
    return lane;
}

void Road::scan(double s, double d, vector<vector<double> > const &sensor_fusion, bool is_debug) {
  /*
  Maps the state of each lane :
    Speed and distance of vehicles ahead
    Speed and distance of vehicles behind
  */
  this->ego_lane = get_lane(d);

  for (int i = 0; i < sensor_fusion.size(); i++) {
      // id, x, y, vx, vy, s, d
      int lane = get_lane(sensor_fusion[i][6]);

      if (lane > -1){
        // In real case scenario, I would have to deal with oncoming traffic
        double distance = sensor_fusion[i][5] - s;

        if (this->lane_front_distance[lane] >= distance & distance >= 0){
          // Closest vehicle ahead so far
          double speed = pow((double)sensor_fusion[i][3], 2) + pow((double)sensor_fusion[i][4], 2);
          speed = sqrt(speed);
          this->lane_front_speed[lane] = speed;
          this->lane_front_distance[lane] = distance;
        }

        if (this->lane_back_distance[lane] <= distance & distance < 0){
          // Closest vehicle behind so far
          double speed = pow((double)sensor_fusion[i][3], 2) + pow((double)sensor_fusion[i][4], 2);
          speed = sqrt(speed);
          this->lane_back_speed[lane] = speed;
          this->lane_back_distance[lane] = distance;
        }

      }
  }

  if (is_debug){
    cout << "Car @ " << this->ego_lane << endl;
    for (int i = 0; i < this->nlanes; i++){
      cout << "Lane : " << i << endl;
      cout << "    Ahead v, s : " << this->lane_front_speed[i] << ", " << this->lane_front_distance[i] << endl;
      cout << "   Behind v, s : " << this->lane_back_speed[i] << ", " << this->lane_back_distance[i] << endl;
    }
  }

}
