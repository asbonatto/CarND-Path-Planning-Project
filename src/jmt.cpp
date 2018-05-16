#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

JMT::~JMT() {};
JMT::JMT(){};

double JMT::get_point(double T){
  return polyval(s, T);
}

double JMT::polyval(vector <double> p, double x){
  double y = 0;
  for (int i = 0; i < p.size(); i++){
    y+= p[i] * pow(x, i);
  }
  return y;
}

vector<double> polyder(vector<double> p){
  // Returns the derivative of a polynomial
  vector<double> der;
  for (int i = 1; i < p.size(); i++){
    der.push_back((i - 1)*p[i]);
  }
  return der;
}

vector<double> JMT::fit(vector<double> istate, vector<double> fstate, double T, bool set_jerk){
    /*
    Returns the coefficients [a0, ... a5] of the Jerk Minimizing JMT
    connecting the initial state [s, s_dot, s_double_dot]
    to the final state in time T.
                  [s, s_dot, s_ddot]
      if set_jerk [s_dot, s_ddot, s_dddot]
    */

    MatrixXd a(3,3);
    VectorXd b(3);

    double T2 =  T*T;
    double T3 = T2*T;
    double T4 = T3*T;
    double T5 = T4*T;

    a <<  T3,    T4,    T5,
        3*T2,  4*T3,  5*T4,
         6*T, 12*T2, 20*T3;

    b << fstate[0] - (istate[0] + istate[1]*T + 0.5*istate[2]*T2),
         fstate[1] - (            istate[1]   +     istate[2]*T ),
         fstate[2] - (                                 istate[2]);

   if (set_jerk){
     a(0, 0) = 6;
     a(0, 1) = 24*T;
     a(0, 2) = 60*T4;
     b(0) = 0;
    }

    VectorXd alpha =a.inverse() * b;

    return {istate[0], istate[1], istate[2]/2, alpha[0], alpha[1], alpha[2]};
}

void JMT::match_lv(vector<double> istate, vector<double> leading_vehicle, bool is_debug){
  /*
  Generates a JMT JMT for matching leading vehicle speed
  */

  // Calculating the s end point according to the leading vehicle kinematics
  double SAFETY_DISTANCE = 10. ;
  vector<double> fstate = {leading_vehicle[0], leading_vehicle[1], 0};

  // Loop for finding the lowest time keeping safety distance
  bool will_collide = true;
  int MAX_ITER = 10;
  int iter = 0;

  while(will_collide){
    cout << "Maneuver duration, max distance : " << duration << " | ";

    double sl = max(leading_vehicle[0] - SAFETY_DISTANCE, SAFETY_DISTANCE);
    for (int i = 1; i < leading_vehicle.size(); i++){
       sl += leading_vehicle[i] * pow(duration, i);
    }
    cout << sl << endl;

    s = fit(istate, fstate, duration, true);

    will_collide = polyval(s, duration) > sl and iter < MAX_ITER;
    duration = duration + 1.0;
    iter++;
  }

  if (is_debug or iter >= MAX_ITER){
    cout << "JMT CONSTRAINTS : " << endl;
    cout << istate[0] << " " << istate[1] << " " << istate[2] << " " << " | ";
    cout << fstate[0] << " " << fstate[1] << " " << fstate[2] << " " << endl;

    cout << "JMT Coeffs | " << duration << endl;
    for (int i = 0; i < s.size(); i++){
      cout << s[i] << ", ";
    }
    cout << endl;
  }
}
