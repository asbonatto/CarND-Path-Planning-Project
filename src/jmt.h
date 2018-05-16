#ifndef JMT_H
#define JMT_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"
#include <vector>

using namespace std;

class JMT{
public:

  vector<double> s;
  double duration = 3.0; // Standard maneuver duration

  JMT();
  ~JMT();

  // Polynomial helper methods
  double polyval(vector <double> p, double x);
  vector<double> polyder(vector<double> p);

  // Trajectory generation methods
  vector<double> fit(vector<double> istate, vector<double> fstate, double T, bool set_jerk = false);
  void match_lv(vector<double> istate, vector<double> leading_vehicle, bool is_debug);

  double get_point(double T);

};
#endif
