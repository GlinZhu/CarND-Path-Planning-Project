#include<iostream>
#include<iterator>
#include<math.h>
#include<algorithm>
#include<string>
#include<vector>
#include<map>
//#include <algorithm>
//include all head files 
#include "Planner.h" //include all functions
#include "Costs.h"
#include "Constant.h"
//using std::map;
using std::vector;
using std::string;
using std::cout;
using std::endl;
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double d, double v, double a, string state){
    this->lane=lane;
    this->s=s;
    this->d=d;
    this->v=v;
    this->a=a;
    this->state=state;
    //max_accel=10; //define the max accel for vehicle, and move it to constant.h
}

Vehicle::~Vehicle(){}
vector<vector<double>> Vehicle::get_best_traj(vector<vector<double>> &predictions, double duration){
    Vehicle vel_ahead;
    vector<string> states=next_available_states(predictions, lane);
    vector<float> total_cost;
    //vector<vector<vector<double>>> final_traj;
    vector<vector<vector<double>>> final_target;
    string best_state;
    vector<bool> car_detected;
    bool car_left=false, car_right=false;
    car_detected=detect_otherCar_beside(predictions); 
    if (car_detected[2]) cout << "CAR ON THE RIGHT!!!" << std::endl;
	if (car_detected[0]) cout << "CAR ON THE LEFT!!!" << std::endl;
	if (car_detected[1]) cout << "CAR JUST AHEAD!!!" << std::endl;
    //vector<vector<double>> target_s_d=select_target_sd(predictions, states[1], duration);
    //final_target.push_back(target_s_d);
    //vector<vector<double>> s_d_history=generate_trajectory(target_s_d, duration);
    //final_traj.push_back(s_d_history);
    //double final_cost=total_costs(s_d_history, target_s_d, duration);
    //total_cost.push_back(final_cost);
    //for(vector<string>::iterator it=states.begin();it!=states.end();++it){
    //    vector<vector<double>> target_s_d=select_target_sd(predictions, *it, duration);
    //    final_target.push_back(target_s_d);
    //    if(target_s_d[0].size()!=0){
            //vector<vector<double>> s_d_history=generate_trajectory(target_s_d, duration);
            //final_traj.push_back(s_d_history);
            //double final_cost=total_costs(s_d_history, target_s_d, duration);
    //        total_cost.push_back(final_cost);
    //    }
    //}
    //vector<float>::iterator best_cost=std::min_element(begin(total_cost), end(total_cost));
    //int best_idx=std::distance(begin(total_cost), best_cost);
    //vector<vector<double>> Best_target=final_target[best_idx];
    //vector<vector<double>> best_traj=final_traj[best_idx];
    //best_state=states[best_idx];
    //vector<vector<double>> Best_target=target_s_d;
    //best_state=states[1];
    //this->state=best_state;
    
    //vector<vector<double>> target_s_d=select_target_sd(predictions, state, duration);
    vector<vector<double>> Best_target;
    // debug
    std::cout<<"Best target of s :"<<Best_target[0][0]<<" "<<Best_target[0][1]<<" "<<Best_target[0][2]<<std::endl;
    std::cout<<"Best target of d :"<<Best_target[1][0]<<" "<<Best_target[1][1]<<" "<<Best_target[1][2]<<std::endl;
    std::cout<<"Chosen state :"<<best_state<<std::endl;
    return Best_target;   // contains {s_target, d_target}
}

vector<string> Vehicle::next_available_states(vector<vector<double>> predictions, int lane){
    vector<string> states;
    states.push_back("KL");
    string state=this->state;
    //Vehicle vel_ahead;
    vector<bool> car_detected;
    bool car_left=false, car_right=false;
    car_detected=detect_otherCar_beside(predictions); //check if there is any vehicle besides
    car_left=car_detected[0];
    car_right=car_detected[2];
    if(car_detected[1]){
        if(this->d>4&&!car_left){
            states.push_back("LCL");
        }
        if(this->d<8&&!car_right){
            states.push_back("LCR");
        }
    }
    else{
        states.push_back("KL");
    }
    //debug
    std::cout<<"the size of states are: "<<states.size()<<std::endl;
    std::cout<<"the available states are: "<<states[0]<<" "<<states[1]<<" "<<states[2]<<std::endl;
    return states;
}

/*
vector<string> Vehicle::next_available_states(vector<vector<double>> predictions, int lane, Vehicle &rvehicle){
    vector<string> states;
    states.push_back("KL");
    string state=this->state;
    Vehicle vel_ahead;
    if(this->d>4){
        states.push_back("LCL");
    }
    if(this->d<8){
        states.push_back("LCR");
    }
    //debug
    std::cout<<"the size of states are: "<<states.size()<<std::endl;
    std::cout<<"the available states are: "<<states[0]<<" "<<states[1]<<" "<<states[2]<<std::endl;
    return states;
}
*/

vector<vector<double>> Vehicle::select_target_sd(vector<vector<double>> &predictions, string state, double duration){
    vector<Vehicle> trajectory;
    vector<double> s_target, d_target;
    if(state.compare("LCL")==0||state.compare("LCR")==0){
        trajectory=lane_change_trajectory(predictions, state, duration);
    }
    else if(state.compare("KL")==0){
        trajectory=lane_keep_trajectory(predictions, duration);
    }
    else if(state.compare("CS")==0){
        trajectory=const_speed(duration);
    }
    s_target={trajectory[1].s, trajectory[1].v, trajectory[1].a};
    d_target={trajectory[1].d, 0, 0};
    return {s_target, d_target};
}
/*
vector<vector<double>> Vehicle::generate_trajectory(vector<vector<double>> target_s_d, double duration){
    vector<double> s_history, d_history;
    vector<double> s_coef, d_coef;
    vector<double> s_start={this->s, this->v, this->a};
    vector<double> d_start={this->d, this->d_dot, this->d_ddot};
    vector<double> s_end=target_s_d[0];
    vector<double> d_end=target_s_d[1];
    //s_coef=JMT(s_start, s_end, duration);
    //d_coef=JMT(d_start, d_end, duration);
    double N=25;    //define there is 25 samples
    for(int i=0; i<N; ++i){
        double t=i*duration/N;
        double s_val(0), d_val(0);
        double pt_s=s_coef[0]+s_coef[1]*i+s_coef[2]*i*i+s_coef[3]*pow(i,3)+s_coef[4]*pow(i,4)+s_coef[5]*pow(i,5);
        s_val+=pt_s;
        double pt_d=s_coef[0]+s_coef[1]*i+s_coef[2]*i*i+s_coef[3]*pow(i,3)+s_coef[4]*pow(i,4)+s_coef[5]*pow(i,5);
        d_val+=pt_d;
        s_history.push_back(s_val);
        d_history.push_back(d_val);
    }
    return {s_history, d_history};

}
*/


vector<double> Vehicle::get_kinematic(vector<vector<double>> &predictions, int lane, double duration){

    double expected_accel_vel=this->v+2*duration;
    double new_position;
    double new_v;
    double new_accel;
    Vehicle vel_ahead;
    Vehicle vel_behind;
    if(get_vehicle_ahead(predictions, lane, vel_ahead)){
        //double max_ahead_vel=(vel_ahead.s-this->s-DIST_BUFFER)/duration+vel_ahead.v;
        new_v=std::min(std::min(vel_ahead.v, expected_accel_vel), MAX_SPEED);
        //if((vel_ahead.s-this->s)<DIST_BUFFER){
        //    new_v=vel_ahead.v;
        //    new_v-=1;
        //}
            
    }
    else{
        new_v=std::min(expected_accel_vel, MAX_SPEED);
    }
    //new_accel=(new_v-this->v)/duration;
    new_accel=0; // set to zero?
    //new_position=this->s+new_v*duration+0.5*new_accel*duration*duration;
    new_position=this->s+(new_v+this->v)/2*duration;
    std::cout<<"the new kinematics are: "<<new_position<<" "<<" "<<new_v<<" "<<new_accel<<std::endl;
    return {new_position, new_v, new_accel};
}
vector<Vehicle> Vehicle::const_speed(double duration){
    vector<Vehicle> trajectory;
    trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state));
    double new_pos=this->s+this->v*duration+0.5*this->a*duration*duration;
    trajectory.push_back(Vehicle(this->lane, new_pos, this->d, this->v, 0, this->state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_keep_trajectory(vector<vector<double>> &predictions, double duration){
    vector<Vehicle> trajectory;
    trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state));
    vector<double> kinematic=get_kinematic(predictions, this->lane, duration);
    trajectory.push_back(Vehicle(this->lane, kinematic[0], this->d, kinematic[1], kinematic[2], "KL"));
    return trajectory;

}
 
vector<Vehicle> Vehicle::lane_change_trajectory(vector<vector<double>> &predictions, string state, double duration){
    vector<Vehicle> trajectory;
    //trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    int new_lane;
    new_lane=this->lane+lane_direction[state];
    Vehicle next_lane_vehicle;
    /*
    for(auto i=0;i<predictions.size();++i){
        vector<bool> car_detected;
        car_detected=detect_otherCar_beside(predictions[i]);
        int rlane=get_lane_val(predictions[i]);
        double r_s=predictions[i][5];
        //check if lane change is possible
        if(state.compare("LCL")==0){
            if(car_detected[0]&&fabs(this->s-r_s)<1.5){   //if car_left is true
                trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state));
                double new_pos=this->s+this->v*duration+0.5*this->a*duration*duration;
                trajectory.push_back(Vehicle(this->lane, new_pos, 2.0+4.0*this->lane, this->v, 0, this->state));
                
            }
            else{
                vector<double> new_kinematic=get_kinematic(predictions, new_lane, duration);
                trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state));
                trajectory.push_back(Vehicle(new_lane, new_kinematic[0], 2.0+4.0*new_lane, new_kinematic[1], new_kinematic[2], state));
                
            }
        }

        if(state.compare("LCR")==0){
            if(car_detected[2]&&fabs(this->s-r_s)<1.5){   //if car_left is true
                trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state));
                double new_pos=this->s+this->v*duration+0.5*this->a*duration*duration;
                trajectory.push_back(Vehicle(this->lane, new_pos, 2.0+4.0*this->lane, this->v, 0, this->state));
                
            }
            else{
                vector<double> new_kinematic=get_kinematic(predictions, new_lane, duration);
                trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state));
                trajectory.push_back(Vehicle(new_lane, new_kinematic[0], 2.0+4.0*new_lane, new_kinematic[1], new_kinematic[2], state));
                
            }
        }
        
    }
    */
   double new_d= 2.0+4.0*new_lane;
    vector<double> new_kinematic=get_kinematic(predictions, new_lane, duration);
    trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state));
    trajectory.push_back(Vehicle(new_lane, new_kinematic[0], new_d, new_kinematic[1], new_kinematic[2], state));
                
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
// Not implemented in the code
bool Vehicle::get_vehicle_behind(vector<vector<double>> predictions, int lane, Vehicle &rvehicle){
    bool found=false;
    Vehicle temp_vehicle;
    int max_s=-1;
    for(auto i=0;i<predictions.size();++i){
        //temp_vehicle=predictions[i];
        vector<bool> car_detected;
        car_detected=detect_otherCar_beside(predictions);
        int rlane=get_lane_val(predictions[i]);
        bool car_inLane=car_detected[1];
        if(car_inLane&&predictions[i][5]<this->s&&predictions[i][5]>max_s){
            found=true;
            max_s=predictions[i][5];   //pass the reference vehicle next time if vehicle 
                                        // behind is found
            double vx=predictions[i][3];
            double vy=predictions[i][4];
            double r_v=sqrt(vx*vx+vy*vy);
            double r_a=0; //assume the predicted vehicle has constant speed
            rvehicle=Vehicle(rlane, predictions[i][5],predictions[i][6], r_v, r_a, "CS");
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
        vector<bool> car_detected;
        car_detected=detect_otherCar_beside(predictions);
        bool car_inLane=car_detected[1];
        int rlane=get_lane_val(predictions[i]);
        if(car_inLane&&predictions[i][5]>this->s&&predictions[i][5]<min_s){
            found=true;
            min_s=predictions[i][5];   //pass the reference vehicle next time if vehicle 
                                        // behind is found
            double vx=predictions[i][3];
            double vy=predictions[i][4];
            double r_v=sqrt(vx*vx+vy*vy);
            double r_a=0; //assume the predicted vehicle has constant speed
            rvehicle=Vehicle(rlane, predictions[i][5],predictions[i][6], r_v, r_a, "CS");
        }
    }
    return found;

}

vector<bool> Vehicle::detect_otherCar_beside(vector<vector<double>> &predictions){
    bool car_right=false, car_left=false, car_ahead=false;
    for(auto i=0;i<predictions.size();++i){
        double otherCar_d=predictions[i][6];
        double r_s=predictions[i][5];
        double d_diff=otherCar_d-this->d;
        if(d_diff>2&&d_diff<6&&(fabs(this->s-r_s))<SAFETYGAP){
            car_right=true;
        }
        else if(d_diff>-6&&d_diff<-2&&(fabs(this->s-r_s))<SAFETYGAP){
            car_left=true;
        }
        else if(d_diff>-2&&d_diff<2&&(r_s-this->s)<DIST_BUFFER){
            car_ahead=true;
        }
    }
    return {car_left, car_ahead, car_right};
}



