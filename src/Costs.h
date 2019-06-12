#ifndef COSTS_H
#define COSTS_H

#include<iostream>
#include<map>
#include<string>
#include<vector>
#include "Planner.h"
#include "Constant.h"
using std::vector;
using std::string;
using std::map;
// define sigmoid function
double sigmoid(double x){
    return 2.0/(exp(-x)+1)-1;
}



// Define cost functions
double efficiency_cost(vector<vector<double>> &trajectory, double duration){
    vector<double> s_traj=trajectory[0];
    vector<double> d_traj=trajectory[1];
    vector<double> ave_v;
    for(unsigned i=1; i<s_traj.size();++i){
        double tag_v=(s_traj[i]-s_traj[i-1])/TIMESTEP;
        ave_v.push_back(tag_v);
    }
    
    return sigmoid(2*MAX_SPEED/ave_v[s_traj.size()-1]-2);
}

double traj_diff_cost(vector<double> s_traj, vector<double> target_s) {
  // Penalizes trajectories whose s coordinate (and derivatives) differ from the goal. Target is s, s_dot, and s_ddot.
  // can be used for d trajectories as well (or any other 1-d trajectory)

  int s_end = s_traj.size();
  double s1, s2, s3, s_dot1, s_dot2, s_ddot, cost = 0;
  s1 = s_traj[s_end - 1];
  s2 = s_traj[s_end - 2];
  s3 = s_traj[s_end - 3];
  s_dot1 = (s1 - s2) / TIMESTEP;
  s_dot2 = (s2 - s3) / TIMESTEP;
  s_ddot = (s_dot1 - s_dot2) / TIMESTEP;
  cost += fabs(s1 - target_s[0]) / 10;
  cost += fabs(s_dot1 - target_s[1]) / 3;
  cost += fabs(s_ddot - target_s[2]) / 0.1;
  return sigmoid(cost);
}

double not_middle_lane_cost(vector<double> d_traj) {
  // penalize not shooting for middle lane (d = 6)
  double end_d = d_traj[d_traj.size()-1];
  return sigmoid(pow(end_d-6, 2));
}


double total_costs(vector<vector<double>> &trajectory, vector<vector<double>> s_d_target, double duration){
  vector<double> s_traj;
  s_traj=trajectory[0];
  float efficient_cost_weight=10000;
  float traj_diff_cost_weight=100;
  double middle_lane_weight=100;
  double total=0;
  total+=efficient_cost_weight*efficiency_cost(trajectory, duration);
  total+=traj_diff_cost_weight*traj_diff_cost(s_traj, s_d_target[0]);
  total+=middle_lane_weight*not_middle_lane_cost(s_d_target[1]);
  return total;
}




float get_lane_speed(vector<vector<double>> &predictions, int lane){
    Vehicle vehicle;
    for (unsigned i=0;i<predictions.size();++i) {
    int key = predictions[i][0];
    int rlane=vehicle.get_lane_val(predictions[i]);
    double vx=predictions[i][3];
    double vy=predictions[i][4];
    double r_v=sqrt(vx*vx+vy*vy);
    if (rlane == lane && key != -1) {
      return r_v;
    }
  }
  // Found no vehicle in the lane
  return -1.0;
}



#endif