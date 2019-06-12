#ifndef JMT_H
#define JMT_H

#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
//#include "helpers.h"
#include <math.h>
#include "helpers.h"
#include "spline.h"
// for convenience
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
double EvalTraj(double x, std::vector<double> coeffs) {
  double y = 0;
  for (int i = 0; i < coeffs.size(); ++i) {
    y += coeffs[i] * pow(x, i);
  }
  return y;
}

/**
 * Differentiate a polynomial with form y = a0 + a1*x + a2*x^2 + ... to get
 * the derivative polynomial's coefficients
 */
std::vector<double> DiffTraj(std::vector<double> coeffs) {
  std::vector<double> diff_coeffs;
  for (int i = 1; i < coeffs.size(); ++i) {
    diff_coeffs.push_back(i*coeffs[i]);
  }
  return diff_coeffs;
}

std::vector<std::vector<double>> InterpolateMap(std::vector<double> map_s,std::vector<double> map_x,std::vector<double> map_y,std::vector<double> map_dx,std::vector<double> map_dy, double s_dist_inc) {
  
  // Add initial map point back to end of map points for wrap-around
  map_s.push_back(kMaxS);
  map_x.push_back(map_x[0]);
  map_y.push_back(map_y[0]);
  map_dx.push_back(map_dx[0]);
  map_dy.push_back(map_dy[0]);
  
  // Make splines by s axis
  tk::spline spline_s_x;
  tk::spline spline_s_y;
  tk::spline spline_s_dx;
  tk::spline spline_s_dy;
  spline_s_x.set_points(map_s, map_x);
  spline_s_y.set_points(map_s, map_y);
  spline_s_dx.set_points(map_s, map_dx);
  spline_s_dy.set_points(map_s, map_dy);
  
  // Interpolate map from splines for s at every s_dist_inc increment
  std::vector<double> interp_s;
  std::vector<double> interp_x;
  std::vector<double> interp_y;
  std::vector<double> interp_dx;
  std::vector<double> interp_dy;
  for (double s=0.; s < kMaxS; s = s + s_dist_inc) {
    interp_s.push_back(s);
    interp_x.push_back(spline_s_x(s));
    interp_y.push_back(spline_s_y(s));
    interp_dx.push_back(spline_s_dx(s));
    interp_dy.push_back(spline_s_dy(s));
  }
  
  return {interp_s, interp_x, interp_y, interp_dx, interp_dy};
}






std::vector<double> GetHiResXY(double s, double d,const std::vector<double> &map_s, const std::vector<double> &map_x,const std::vector<double> &map_y) {
  
  // Wrap around s
  
  s = std::fmod(s, kMaxS);

  // Find 2 waypoints before s and 2 waypoints after s for angular interpolation
  auto it_wp1_search = std::lower_bound(map_s.begin(), map_s.end(), s);
  int wp1 = (it_wp1_search - map_s.begin() - 1); // wp before s
  int wp2 = (wp1 + 1) % map_s.size(); // wp after s
  int wp3 = (wp2 + 1) % map_s.size(); // wp 2nd after s
  int wp0 = wp1 - 1; // wp 2nd before s
  if (wp0 < 0) { wp0 = map_s.size() - 1; } // wrap around backwards

  // Use angle between wp1-wp2 to derive segment vector at distance s from wp1
  double theta_wp = atan2((map_y[wp2] - map_y[wp1]),
                          (map_x[wp2] - map_x[wp1]));
  
  // The (x,y,s) along the segment vector between wp1 and wp2
  double seg_s = s - map_s[wp1];
  double seg_x = map_x[wp1] + seg_s * cos(theta_wp);
  double seg_y = map_y[wp1] + seg_s * sin(theta_wp);

  // Interpolate theta at s based on the distance between wp1 (with ave angle
  // from wp0 before and wp2 after) and wp2 (with ave angle from wp1 before
  // and wp3 after)
  double theta_wp1ave = atan2((map_y[wp2] - map_y[wp0]),
                              (map_x[wp2] - map_x[wp0]));
  
  double theta_wp2ave = atan2((map_y[wp3] - map_y[wp1]),
                              (map_x[wp3] - map_x[wp1]));
  
  double s_interp = (s - map_s[wp1]) / (map_s[wp2] - map_s[wp1]);
  
  double cos_interp = ((1-s_interp) * cos(theta_wp1ave)
                         + s_interp * cos(theta_wp2ave));
  
  double sin_interp = ((1-s_interp) * sin(theta_wp1ave)
                         + s_interp * sin(theta_wp2ave));
  
  double theta_interp = atan2(sin_interp, cos_interp);
  
  // Use interpolated theta to calculate final (x,y) at d offset from the
  // segment vector
  double theta_perp = theta_interp - pi()/2;
  double x = seg_x + d * cos(theta_perp);
  double y = seg_y + d * sin(theta_perp);
  
  return {x, y};
}


#endif
