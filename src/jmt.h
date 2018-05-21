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

  vector<double> s, v, a;

  JMT();
  ~JMT();

  // Polynomial helper methods
  double polyval(vector <double> p, double x);
  vector<double> polyder(vector<double> p);
  void polyprint(vector <double> p);

  // Trajectory generation methods
  void fit(vector<double> istate, vector<double> fstate, double T, bool set_jerk = false);
  void derivatives();
  vector<double> get_sva(double T);

};
#endif
