#ifndef PLANNER_H
#define PLANNER_H

#include<iostream>
#include<map>
#include<string>
#include<vector>

using std::vector;
using std::string;
using std::map;

class Vehicle{
    public:
    //constructor
    Vehicle();
    Vehicle(int lane, double s, double v, double a, string state="CS");
    //destructor
    virtual ~Vehicle();

    //functions for behavior planner
    //construct functions for FSM
    vector<Vehicle> choose_next_state(vector<vector<double>> &predictions);
    vector<string> next_states();
    vector<Vehicle> generate_trajectory(vector<vector<double>> &predictions, string state);
    vector<Vehicle> const_speed();
    vector<Vehicle> lane_keep_trajectory(vector<vector<double>> &predictions);
    vector<Vehicle> lane_change_trajectory(vector<vector<double>> &predictions, string state);
    vector<Vehicle> pre_lane_change_trajectory(vector<vector<double>> &predictions, string state);
    vector<double> get_kinematic(vector<vector<double>> &predictions, int lane);
    int get_lane_val(vector<double> &sensor_fusion);
    bool get_vehicle_behind(vector<vector<double>> predictions, int lane, Vehicle &rvehicle);
    bool get_vehicle_ahead(vector<vector<double>> predictions, int lane, Vehicle &rvehicle);
    map<string, int> lane_direction={{"PLCL", 1}, {"LCL", 1}, {"PLCR", -1}, {"LCR", -1}};
    map<string, int> Road_lane={{"left", 0}, {"middle", 1}, {"right", 2}};
    struct collider{
        bool collision;
        float time;
    };

    //define all class member data
    int lane, goal_lane;
    double s, d, v, a, goal_s, target_v, max_accel;
    string state;
    float dist_buffer=10.0; // the safety distance for lane keeping and const speed trajectory


};

// create the cost functions in this folder
float inefficiency_cost(vector<vector<double>> &predictions, Vehicle &vehicle, vector<Vehicle> &trajectory);
float get_lane_speed(vector<vector<double>> &predictions, int lane);
map<string, int> get_lane_data(Vehicle &vehicle, vector<vector<double>> &predictions, vector<Vehicle> &trajectory);

#endif  // PLANNER_H
