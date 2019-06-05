#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
//#include "helpers.h"
#include <math.h>
#include "Traj_generation.h"
// for convenience
using std::string;
using std::vector;
using std::map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
/*
vector<double> JMT(vector<double> &start, vector<double> &end, double T){
   double a0=start[0];
   double a1=start[1];
   double a2=0.5*start[2];
   double a3, a4, a5;
   vector<double> coeff;
   VectorXd alpha(3);
   alpha(0)=a3;
   alpha(1)=a4;
   alpha(2)=a5;
   MatrixXd T_(3,3);
   T_ << pow(T,3), pow(T,4), pow(T,5),
        3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
        6*T, 12*pow(T,2), 20*pow(T,3);
   double s1=end[0]-(start[0]+start[1]*T+0.5*start[2]*pow(T,2));
   double s2=end[1]-(start[1]+start[2]*T);
   double s3=end[2]-start[2];
   VectorXd Solution(3);
   Solution<< s1, s2, s3;
   alpha=T_.inverse()*Solution;
   coeff={a0, a1, a2};
   for(int i=0;i<3;++i){
	   coeff.push_back(alpha.data()[i]);
   }
   return coeff;
	
}
*/

vector<double> JMT(vector<double> &start, vector<double> &end, double T){
   MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
	
}


