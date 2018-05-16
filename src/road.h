#include <math.h>
#include <vector>
#include <string>
#include <iterator>
#include <limits>

using namespace std;

class Road {
public:

	int nlanes;
	int ego_lane;
	double speed_limit;
	double lane_width;
	vector<double> lane_front_speed, lane_front_distance;
	vector<double> lane_back_speed, lane_back_distance;

	Road(double speed_limit, int nlanes, double lane_width);
	virtual ~Road();

	int get_lane(double d);
	void scan(double s, double d, vector<vector<double> > const &sensor_fusion, bool is_debug = false);

};
