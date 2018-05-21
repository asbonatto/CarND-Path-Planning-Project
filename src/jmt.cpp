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
