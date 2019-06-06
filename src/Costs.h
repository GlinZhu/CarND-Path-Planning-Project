#ifndef PLANNER_H
#define PLANNER_H

#include<iostream>
#include<map>
#include<string>
#include<vector>
#include "Planner.h"
using std::vector;
using std::string;
using std::map;

// Define cost functions
float inefficiency_cost(vector<vector<double>> &predictions, Vehicle &vehicle, vector<Vehicle> &trajectory){
    map<string, int> lane_data=get_lane_data(vehicle, predictions, trajectory);
    float proposed_speed_intended = get_lane_speed(predictions, lane_data["intended_lane"]);
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = vehicle.target_v;
  }

    float proposed_speed_final = get_lane_speed(predictions, lane_data["final_lane"]);
    if (proposed_speed_final < 0) {
        proposed_speed_final = vehicle.target_v;
  }
    
    float cost = (2.0*vehicle.target_v - proposed_speed_intended 
             - proposed_speed_final)/vehicle.target_v;

    return cost;
}


map<string, int> get_lane_data(Vehicle &vehicle, vector<vector<double>> &predictions, vector<Vehicle> &trajectory){
    map<string, int> trajectory_data;
    Vehicle trajectory_last=trajectory[1];
    int intended_lane;
    if(trajectory_last.state.compare("PLCL")==0){
        intended_lane=trajectory_last.lane+1;
    }
    else if(trajectory_last.state.compare("PLCR")==0){
        intended_lane=trajectory_last.lane-1;
    }
    else{
        intended_lane=trajectory_last.lane;
    }
    int final_lane=trajectory_last.lane;
    trajectory_data["intended_lane"]=intended_lane;
    trajectory_data["final_lane"]=final_lane;

    return trajectory_data;

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