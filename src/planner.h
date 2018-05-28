#include <math.h>
#include <vector>
#include <string>
#include <iterator>
#include <limits>
#include "spline.h"
#include "csys.h"
#include "jmt.h"

using namespace std;

class Planner {
public:

	// Car constraints
	double MAX_SPEED = mph2mps(47.5);
	double MAX_ACCEL = 10; // m/s^2
	double MAX_JERK  = 10; // m/s^3

	// Coordinate system and map utility
	CSys* csys;

	// Behavior Planner
	int target_lane;
	int current_lane;
	double T = 1.0; // prediction horizon

	// Traffic info 
	vector<vector<double>> vehicle_ahead, vehicle_behind;
	vector<double> lane_speeds = {MAX_SPEED, MAX_SPEED, MAX_SPEED};
	vector<double> lane_costs = {MAX_SPEED, MAX_SPEED, MAX_SPEED};

	double inf = numeric_limits<double>::infinity();

	// Ego vehicle ground truth data
	double grd_s, grd_d, grd_vs, grd_vd;

	// Trajectory data
	vector<double> path_s, path_d, path_x, path_y;
	vector<double> vec_s, vec_d;
	JMT traj_s;
  	JMT traj_d;

	// Trajectory generation data
	double dt = 20./1000.; // planner clock ...
	int NPTS = (int) (T / dt);
	
	// Methods
	Planner(CSys*);
	virtual ~Planner(){};

	// Engineering utilities
	double mph2mps(double mph) {return mph*0.44704;}
	double deg2rad(double x) { return x * M_PI / 180; }
	double rad2deg(double x) { return x * 180 / M_PI; }

	// Sensor fusion updates
	void update_telemetry(double x, double y, double s, double d, double yaw, double speed, int unused_pts);
	void scan_road(vector<vector<double> > const &sensor_fusion, bool is_debug = false);

	// Behavior planner functions
	vector<int> get_available_lanes(int lane);
	void generate_trajectory(int unused_pts);
	int choose_lane();

	// Helper functions that may get dumped
	double get_lane_speed(int lane);
	void update_lane_costs();
	double time_to_collision(int lane);
	
	double logit(double x){return 1.0/(1.0 + exp(-x));}
};
