#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

JMT::JMT(vector<double> p, double speed, double accel, double jerk){
  MAX_SPEED = speed;
  MAX_ACCEL = accel;
  MAX_JERK = jerk;
  for (int i = 0; i < p.size(); i++)
  {
    s.push_back(p[i]);
  }
  
  v = polyder(s);
  a = polyder(v);
};

vector<double> JMT::get_sva(double T){
  return {polyval(s, T), polyval(v, T), polyval(a, T)};
}

double JMT::polyval(vector <double> p, double x){
  double y = 0;
  for (int i = 0; i < p.size(); i++){
    y+= p[i] * pow(x, i);
  }
  return y;
}

vector<double> JMT::polyder(vector<double> p){
  // Returns the derivative of a polynomial
  vector<double> der;
  double coeff;
  for (int i = 1; i < p.size(); i++){
    coeff = p[i] * i;
    der.push_back(coeff);
  }
  return der;
}

void JMT::fit(vector<double> istate, vector<double> fstate, double T, bool set_jerk){
    /*
    Returns the coefficients [a0, ... a5] of the Jerk Minimizing JMT
    connecting the initial state [s, s_dot, s_double_dot]
    to the final state in time T.
                  [s, s_dot, s_ddot]
      if set_jerk [s_dot, s_ddot, s_dddot]
    */

    MatrixXd A(3,3);
    VectorXd b(3);

    // Avoiding exceeding jerk and acceleration limits by mispecification of T
    T = max(T, 2*fabs(fstate[1] - istate[1])/MAX_ACCEL);
    T = max(T, fabs(fstate[2] - istate[2])/MAX_JERK);

    double T2 =  T*T;
    double T3 = T2*T;
    double T4 = T3*T;
    double T5 = T4*T;

    A <<  T3,    T4,    T5,
        3*T2,  4*T3,  5*T4,
         6*T, 12*T2, 20*T3;

    b << fstate[0] - (istate[0] + istate[1]*T + 0.5*istate[2]*T2),
         fstate[1] - (            istate[1]   +     istate[2]*T ),
         fstate[2] - (                                 istate[2]);

   if (set_jerk){
     A(0, 0) = 6;
     A(0, 1) = 24*T;
     A(0, 2) = 60*T4;
     b(0) = 0;
    }

    VectorXd alpha = A.inverse() * b;
    s.clear();
    v.clear();
    a.clear();

    s = {istate[0], istate[1], istate[2]/2, alpha[0], alpha[1], alpha[2]};
    derivatives();

}

void JMT::derivatives(){
  v = polyder(s);
  a = polyder(v);

}

void JMT::polyprint(vector <double> p)
{
  cout << p[0];
  for (int i = 1; i < p.size(); i++)
  {
    cout << ", " << p[i];
  }
  cout << endl;
}
