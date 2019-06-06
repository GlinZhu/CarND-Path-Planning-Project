#include<iostream>
#include<iterator>
#include<math.h>
#include<algorithm>
#include<string>
#include<vector>
#include<map>
//include all head files 
#include "Planner.h" //include all functions
#include "Costs.h"
//using std::map;
using std::vector;
using std::string;
using  std::cout;

Vehicle::Vehicle(){cout<<"create a calss of vehicle\n";}

Vehicle::Vehicle(int lane, double s, double v, double a, string state){
    this->lane=lane;
    this->s=s;
    this->v=v;
    this->a=a;
    this->state=state;
    max_accel=10; //define the max accel for vehicle;
}

Vehicle::~Vehicle(){}
vector<Vehicle> Vehicle::get_target_s_d(vector<vector<double>> &predictions){
    vector<string> states=next_available_states();
    vector<float> total_cost;
    vector<vector<Vehicle>> final_traj;
    for(vector<string>::iterator it=states.begin();it!=states.end();++it){
        vector<Vehicle> traj=generate_trajectory(predictions, *it);
        if(traj.size()!=0){

            final_traj.push_back(traj);
            total_cost.push_back(inefficiency_cost(predictions, *this, traj));
        }
    }
    vector<float>::iterator best_cost=std::min_element(begin(total_cost), end(total_cost));
    int best_idx=std::distance(begin(total_cost), best_cost);
    return final_traj[best_idx];
}

vector<string> Vehicle::next_available_states(){
    vector<string> states;
    states.push_back("KL");
    string state=this->state;
    if(this->lane!=0&&this->d>4){
        states.push_back("LCL");
    }
    if(this->lane!=2&&this->d<8){
        states.push_back("LCR");
    }
return states;
}

vector<Vehicle> Vehicle::generate_trajectory(vector<vector<double>> &predictions, string state){
    vector<Vehicle> trajectory;
    if(state.compare("LCL")==0||state.compare("LCR")==0){
        trajectory=lane_change_trajectory(predictions, state);
    }
    else if(state.compare("PLCL")==0||state.compare("PLCR")==0){
        trajectory=pre_lane_change_trajectory(predictions, state);
    }
    else if(state.compare("KL")==0){
        trajectory=lane_keep_trajectory(predictions);
    }
    else if(state.compare("CS")==0){
        trajectory=const_speed();
    }
    return trajectory;
}
vector<double> Vehicle::get_kinematic(vector<vector<double>> &predictions, int lane, double duration){

    double max_accel_vel=this->v+this->max_accel*duration;
    double new_position;
    double new_v;
    double new_accel;
    Vehicle vel_ahead;
    Vehicle vel_behind;
    if(get_vehicle_ahead(predictions, lane, vel_ahead)){
        if(get_vehicle_behind(predictions, lane, vel_behind)){
            // set new speed to traffic limit
            new_v=vel_ahead.v;
        }
        else{
            double max_ahead_vel=vel_ahead.s-this->s-dist_buffer+vel_ahead.v-0.5*this->a;
            new_v=std::min(std::min(max_ahead_vel, max_accel_vel), this->target_v);
        }
    }
    else{
        new_v=std::min(max_accel_vel, this->target_v);
    }
    new_accel=new_v-this->v;
    new_position=this->s+new_v+0.5*new_accel;
    return {new_position, new_v, new_accel};
}
vector<Vehicle> Vehicle::const_speed(){
    vector<Vehicle> trajectory;
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    double new_pos=this->s+this->v+0.5*this->a;
    trajectory.push_back(Vehicle(this->lane, new_pos, this->v, 0, this->state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_keep_trajectory(vector<vector<double>> &predictions){
    vector<Vehicle> trajectory;
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<double> kinematic=get_kinematic(predictions, this->lane);
    trajectory.push_back(Vehicle(this->lane, kinematic[0], kinematic[1], kinematic[2], "KL"));
    return trajectory;

}
 
vector<Vehicle> Vehicle::lane_change_trajectory(vector<vector<double>> &predictions, string state){
    vector<Vehicle> trajectory;
    //trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    int new_lane=this->lane+lane_direction[state];
    Vehicle next_lane_vehicle;
    for(auto i=0;i<predictions.size();++i){
        int rlane=get_lane_val(predictions[i]);
        double r_s=predictions[i][5];
        //check if lane change is possible
        if(rlane==new_lane&&this->s==r_s){
            return trajectory; //return an empty trajectory;
        }
    }
    vector<double> new_kinematic=get_kinematic(predictions, new_lane);
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    trajectory.push_back(Vehicle(new_lane, new_kinematic[0], new_kinematic[1], new_kinematic[2], state));
    return trajectory;

}

int Vehicle::get_lane_val(vector<double> &sensor_fusion){
    double d=sensor_fusion[6];
    if(d>0&&d<4){
        lane=0;
    }
    else if(d>4&&d<8){
        lane=1;
    }
    else if(d>8&&d<12){
        lane=2;
    }
    return lane;
}

bool Vehicle::get_vehicle_behind(vector<vector<double>> predictions, int lane, Vehicle &rvehicle){
    bool found=false;
    Vehicle temp_vehicle;
    int max_s=-1;
    for(auto i=0;i<predictions.size();++i){
        //temp_vehicle=predictions[i];
        int rlane=get_lane_val(predictions[i]);
        if(rlane==lane&&predictions[i][5]<this->s&&predictions[i][5]>max_s){
            found=true;
            max_s=predictions[i][5];   //pass the reference vehicle next time if vehicle 
                                        // behind is found
            double vx=predictions[i][3];
            double vy=predictions[i][4];
            double r_v=sqrt(vx*vx+vy*vy);
            double r_a=0; //assume the predicted vehicle has constant speed
            rvehicle=Vehicle(rlane, predictions[i][5],r_v, r_a, "CS");
        }
    }
    return found;

}

bool Vehicle::get_vehicle_ahead(vector<vector<double>> predictions, int lane, Vehicle &rvehicle){
    bool found=false;
    Vehicle temp_vehicle;
    int min_s=std::numeric_limits<int>::max();
    for(auto i=0;i<predictions.size();++i){
        //temp_vehicle=predictions[i];
        int rlane=get_lane_val(predictions[i]);
        if(rlane==lane&&predictions[i][5]>this->s&&predictions[i][5]<min_s){
            found=true;
            min_s=predictions[i][5];   //pass the reference vehicle next time if vehicle 
                                        // behind is found
            double vx=predictions[i][3];
            double vy=predictions[i][4];
            double r_v=sqrt(vx*vx+vy*vy);
            double r_a=0; //assume the predicted vehicle has constant speed
            rvehicle=Vehicle(rlane, predictions[i][5],r_v, r_a, "CS");
        }
    }
    return found;

}



