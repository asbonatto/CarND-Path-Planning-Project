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
	double MAX_SPEED = mph2mps(50.0);
	double MAX_ACCEL = 10; // m/s^2
	double MAX_JERK  = 10; // m/s^3

	// Road data
	CSys* csys;
	int target_lane;
	int current_lane;

	vector<double> lane_speeds, lane_distances;
	double inf = numeric_limits<double>::infinity();

	// telemetry data
	double grd_s, grd_d, grd_vs, grd_vd;

	 // State vector for polynomial solution
	vector<double> initial_s;
	vector<double> initial_d;
	vector<double> last_s;
	vector<double> last_d;

	// Trajectory generation parameters
	double dt = 20./1000.; // planner clock ...
	int NPTS = 50;
	vector<double> path_s;
	vector<double> path_d;
	vector<double> path_x;
	vector<double> path_y;

	// Methods
	Planner(CSys*);
	virtual ~Planner(){};

	double mph2mps(double mph) {return mph*0.44704;}
	double deg2rad(double x) { return x * M_PI / 180; }
	double rad2deg(double x) { return x * 180 / M_PI; }

	void update_telemetry(double x, double y, double s, double d, double yaw, double speed, int unused_pts);
	void generate_trajectory(int unused_pts);
	void scan_road(vector<vector<double> > const &sensor_fusion, bool is_debug = true);

};
