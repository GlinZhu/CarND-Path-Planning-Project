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
    Vehicle(int lane, double s, double d, double v, double a, string state="CS");
    //destructor
    virtual ~Vehicle();

    //functions for behavior planner
    //construct functions for FSM
    vector<vector<double>> get_best_traj(vector<vector<double>> &predictions, double duration);
    vector<string> next_available_states(vector<vector<double>> predictions, int lane);
    vector<vector<double>> select_target_sd(vector<vector<double>> &predictions, string state, double duration);
    vector<vector<double>> generate_trajectory(vector<vector<double>> target_s_d, double duration);
    vector<Vehicle> const_speed(double duration);
    vector<Vehicle> lane_keep_trajectory(vector<vector<double>> &predictions, double duration);
    vector<Vehicle> lane_change_trajectory(vector<vector<double>> &predictions, string state, double duration);
    vector<double> get_kinematic(vector<vector<double>> &predictions, int lane, double duration);
    int get_lane_val(vector<double> &sensor_fusion);
    bool get_vehicle_behind(vector<vector<double>> predictions, int lane, Vehicle &rvehicle);
    bool get_vehicle_ahead(vector<vector<double>> predictions, int lane, Vehicle &rvehicle);
    vector<bool> detect_otherCar_beside(vector<vector<double>> &predictions);
    map<string, int> lane_direction={{"LCL", -1}, {"LCR", 1}};
    map<string, int> Road_lane={{"left", 0}, {"middle", 1}, {"right", 2}};
    struct collider{
        bool collision;
        float time;
    };

    //define all class member data
    int lane;
    double s, d, v, a, target_v;
    double d_dot, d_ddot;
    string state; 


};



#endif  // PLANNER_H
