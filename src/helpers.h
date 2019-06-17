#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}



// decide which lane ego vehicle is at

int LaneDetect(double d_pos){
  int lane;
  if(d_pos>0&&d_pos<4){
      lane=0;
  }
  else if(d_pos>4&&d_pos<8){
      lane=1;
  }
  else if(d_pos>8&&d_pos<12){
      lane=2;
  }
  return lane;
}

// Define map function to get high precision of map
int path_interpolate=30;
void sortXY( vector< double> &X_tmp, vector< double> &Y_tmp, int flag)
{
  vector<double> results[2];
  if (flag == 1){
    for (int i = 0; i<4; i++){
      double smallest_X = X_tmp[i];
      int smallest_idx = i;
      for (int j = i+1; j<4; j++){
        if (X_tmp[j] < smallest_X){
          smallest_X = X_tmp[j];
          smallest_idx = j;
        }
      }
      if(smallest_idx!=i){
        double tmp = X_tmp[i];
        X_tmp[i] = X_tmp[smallest_idx];
        X_tmp[smallest_idx] = tmp;
        tmp = Y_tmp[i];
        Y_tmp[i] = Y_tmp[smallest_idx];
        Y_tmp[smallest_idx] = tmp;
      }
    }
  }
  else{
    for (int i = 0; i<4; i++){
      double smallest_Y = Y_tmp[i];
      int smallest_idx = i;
      for (int j = i+1; j<4; j++){
        if (Y_tmp[j] < smallest_Y){
          smallest_Y = Y_tmp[j];
          smallest_idx = j;
        }
      }
      if(smallest_idx!=i){
        double tmp = X_tmp[i];
        X_tmp[i] = X_tmp[smallest_idx];
        X_tmp[smallest_idx] = tmp;
        tmp = Y_tmp[i];
        Y_tmp[i] = Y_tmp[smallest_idx];
        Y_tmp[smallest_idx] = tmp;
      }
    }
  }
}

void map_refine(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
  vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy,
  vector<double> &map_waypoints_x_tmp, vector<double> map_waypoints_y_tmp, vector<double> map_waypoints_s_tmp,
  vector<double> map_waypoints_dx_tmp, vector<double> map_waypoints_dy_tmp){

    double max_s = 6945.554;

    vector<double> X_tmp(4), Y_tmp(4);
    int i1, i2, i3, i4;
    tk::spline s_tmp;

    for (int i =0; i<map_waypoints_x.size();  i++){
      i2 = i;
      i1 = i2 - 1;
      if (i1<0){i1 += map_waypoints_x.size();}
      i3 = i2 + 1;
      if (i3 >= map_waypoints_x.size()){ i3 -= map_waypoints_x.size();}
      i4 = i2 + 2;
      if (i4 >= map_waypoints_x.size()){ i4 -= map_waypoints_x.size();}
      X_tmp[0] = map_waypoints_x[i1];
      X_tmp[1] = map_waypoints_x[i2];
      X_tmp[2] = map_waypoints_x[i3];
      X_tmp[3] = map_waypoints_x[i4];
      Y_tmp[0] = map_waypoints_y[i1];
      Y_tmp[1] = map_waypoints_y[i2];
      Y_tmp[2] = map_waypoints_y[i3];
      Y_tmp[3] = map_waypoints_y[i4];

      if (abs(map_waypoints_x[i2]-map_waypoints_x[i3])/map_waypoints_x[i2] > 0.005){
        sortXY(X_tmp, Y_tmp, 1);
        //cout << setw(25) << "sorted X:  "<< X_tmp[0] << " " << X_tmp[1] << " " << X_tmp[2] << " "<< X_tmp[3] << " " << endl;
        //cout << setw(25) << "sorted Y:  "<< Y_tmp[0] << " " << Y_tmp[1] << " " << Y_tmp[2] << " "<< Y_tmp[3] << " " << endl;
        s_tmp.set_points(X_tmp, Y_tmp);
        for(int i = 0; i<path_interpolate; i++){
          double x = map_waypoints_x[i2] + (map_waypoints_x[i3] - map_waypoints_x[i2])/path_interpolate*i;
          //vector<double> fitting_results = s_tmp(x);
          //double y = fitting_results[0];
          double y = s_tmp(x);
          map_waypoints_x_tmp.push_back(x);
          map_waypoints_y_tmp.push_back(y);
          double s;
          if (map_waypoints_s[i2] < 6900.0){
            s = (map_waypoints_s[i2]*(path_interpolate-i) + map_waypoints_s[i3]*i)/path_interpolate;
          }
          else{
            s = (map_waypoints_s[i2]*(path_interpolate-i) + max_s*i)/path_interpolate;
          }
          map_waypoints_s_tmp.push_back(s);
          //double slope = fitting_results[1];
          //double direct = atan(slope);
          //double direct_perp = atan(slope) - pi()/2;
          double d_x_tmp = (map_waypoints_dx[i2]*(path_interpolate-i) + map_waypoints_dx[i3]*i)/path_interpolate;
          double d_y_tmp = (map_waypoints_dy[i2]*(path_interpolate-i) + map_waypoints_dy[i3]*i)/path_interpolate;
          map_waypoints_dx_tmp.push_back(d_x_tmp/sqrt(d_x_tmp*d_x_tmp + d_y_tmp*d_y_tmp));
          map_waypoints_dy_tmp.push_back(d_y_tmp/sqrt(d_x_tmp*d_x_tmp + d_y_tmp*d_y_tmp));
          //if (dflag >= dflag_fitting) {cout << setw(25) << "fitting:  "<< x << " " << y << " " << s << " " << d_x_tmp << " " << d_y_tmp << endl;}
        }
      }
      else{
        sortXY(X_tmp, Y_tmp, 2);
        //cout << setw(25) << "sorted X:  "<< X_tmp[0] << " " << X_tmp[1] << " " << X_tmp[2] << " "<< X_tmp[3] << " " << endl;
        //cout << setw(25) << "sorted Y:  "<< Y_tmp[0] << " " << Y_tmp[1] << " " << Y_tmp[2] << " "<< Y_tmp[3] << " " << endl;
        s_tmp.set_points(Y_tmp, X_tmp);
        for(int i = 0; i<path_interpolate; i++){
          double y = map_waypoints_y[i2] + (map_waypoints_y[i3] - map_waypoints_y[i2])/path_interpolate*i;
          //vector<double> fitting_results = s_tmp(y);
          //double x = fitting_results[0];
          double x = s_tmp(y);
          map_waypoints_x_tmp.push_back(x);
          map_waypoints_y_tmp.push_back(y);
          double s;
          if (map_waypoints_s[i2] < 6900.0){
            s = (map_waypoints_s[i2]*(path_interpolate-i) + map_waypoints_s[i3]*i)/path_interpolate;
          }
          else{
            s = (map_waypoints_s[i2]*(path_interpolate-i) + max_s*i)/path_interpolate;
          }
          map_waypoints_s_tmp.push_back(s);
          //double slope = fitting_results[1];
          //double direct = atan(slope);
          //double direct_perp = atan(slope) - pi()/2;
          double d_x_tmp = (map_waypoints_dx[i2]*(path_interpolate-i) + map_waypoints_dx[i3]*i)/path_interpolate;
          double d_y_tmp = (map_waypoints_dy[i2]*(path_interpolate-i) + map_waypoints_dy[i3]*i)/path_interpolate;
          map_waypoints_dx_tmp.push_back(d_x_tmp/sqrt(d_x_tmp*d_x_tmp + d_y_tmp*d_y_tmp));
          map_waypoints_dy_tmp.push_back(d_y_tmp/sqrt(d_x_tmp*d_x_tmp + d_y_tmp*d_y_tmp));
          //if (dflag >= dflag_fitting) {cout << setw(25) << "fitting:  "<< x << " " << y << " " << s << " " << d_x_tmp << " " << d_y_tmp << endl;}
        }
      }
    }
    //if (dflag >= dflag_fitting) {cout << setw(25) << "fitting summary:  "<< map_waypoints_x.size() << " " << map_waypoints_x[0] << " " << map_waypoints_y[0] << " " << map_waypoints_s[0] << endl;}
}

void spline_fitting(tk::spline &s_x, tk::spline &s_y, tk::spline &s_dx, tk::spline &s_dy,vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy){

  s_x.set_points(maps_s, maps_x);
  s_y.set_points(maps_s, maps_y);
  s_dx.set_points(maps_s, maps_dx);
  s_dy.set_points(maps_s, maps_dy);

}


#endif  // HELPERS_H