#ifndef COSTS_H
#define COSTS_H

#include<iostream>
#include<map>
#include<string>
#include<vector>
//#include "Planner.h"
#include "Constant.h"
using std::vector;
using std::string;
using std::map;
// define sigmoid function
double sigmoid(double x){
    return 2.0/(exp(-x)+1)-1;
}



// Define cost functions
double efficiency_cost(vector<vector<double>> detected_car_list, double current_s, double current_v, double duration){
  int front_idx=-1;
  for(auto i=0;i<detected_car_list.size();++i){
    double target_car_s=detected_car_list[detected_car_list.size()-i-1][1];
    if((target_car_s-current_s)>0&&(target_car_s-current_s)<current_s+current_v*duration*2){
      if(front_idx==-1){front_idx=detected_car_list.size()-i-1;}
    }
  }
  double tar_car_v=detected_car_list[front_idx][2];
  if(front_idx!=-1){
    if(tar_car_v>MAX_SPEED){
      tar_car_v=MAX_SPEED;
    }
  }
  else{
    tar_car_v=MAX_SPEED;
  }
  return 1-tar_car_v/MAX_SPEED;

}


double check_collision(vector<vector<double>> detected_car_list, double current_s, double current_v, double duration) {
  // check if there is car in the lane while doing lane changing
  int front_idx=-1;
  for(auto i=0;i<detected_car_list.size();++i){
    double target_car_s=detected_car_list[detected_car_list.size()-i-1][1];
    if((target_car_s-current_s)>0){
      if(front_idx==-1){front_idx=detected_car_list.size()-i-1;}
    }
  }
  //there are three cases where all cars in the front of ego_car, and all cars is behind of the ego_car, and ego car is in the middle of detected cars;
  if(front_idx==-1&&detected_car_list.size()!=0){
    double tar_car_s=detected_car_list[detected_car_list.size()-1][1];
    double tar_car_v=detected_car_list[detected_car_list.size()-1][2];
    if(tar_car_s+tar_car_v*duration+0.5*LCBuffer>current_s+current_v*duration){
      return 1.0; //collision
    }
    else{
      return 0.0;
    }
  }
  else if(front_idx==detected_car_list.size()-1&&detected_car_list.size()!=0){
    double tar_car_s=detected_car_list[detected_car_list.size()-1][1];
    double tar_car_v=detected_car_list[detected_car_list.size()-1][2];
    if(tar_car_s+tar_car_v*duration<current_s+current_v*duration+LCBuffer){
      return 1.0; //collision
    }
    else{
      return 0.0;
    }
  }
  else if(front_idx != -1){
    double s1 = detected_car_list[front_idx+1][1];
    double v1 = detected_car_list[front_idx+1][3];
    double s2 = detected_car_list[front_idx][1];
    double v2 = detected_car_list[front_idx][3];
    if( s1+v1*duration+0.5*LCBuffer > current_s+current_v*duration){
      //cout << "CAUTIOUS!!! " << s1 - car_s << " " << v1 - car_speed << endl;
      return 1.0;
    }
    else if( s2+v2*duration < current_s+current_v*duration+LCBuffer ){
      //cout << "CAUTIOUS!!! " << s2 - car_s << " " << v2 - car_speed << endl;
      return 1.0;
    }
    else {return 0.0;}
  }
  else{return 0.0;}

}




double total_costs(vector<vector<double>> detected_car_list, double current_s, double current_v, double duration){
  // define the weights for all cost functions
  double Check_collision=1.0;
  double speed_cost=0.9;
  double total_cost;
  total_cost=Check_collision*check_collision(detected_car_list, current_s, current_v, duration)+speed_cost*efficiency_cost(detected_car_list, current_s, current_v, duration);

  return total_cost;
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