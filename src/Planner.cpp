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

double Vehicle::next_chosen_states(vector<vector<double>> detected_car_left, vector<vector<double>> detected_car_middle, vector<vector<double>> detected_car_right){
    vector<string> states;
    states.push_back("KL");
    string state=this->state;
    //Vehicle vel_ahead;
    if(current_d>8.0){
        states.push_back("LCL");
    }
    else if(current_d<4){
        states.push_back("LCR");
    }
    else{
        states.push_back("LCL");
        states.push_back("LCR");
    }
    //debug
    std::cout<<"the size of states are: "<<states.size()<<std::endl;
    std::cout<<"the available states are: "<<states[0]<<" "<<states[1]<<" "<<states[2]<<std::endl;
    vector<double> costs;
    for(auto i=0; i<states.size(); ++i){
        if(states[i].compare("LCL")==0){
            double current_cost=total_costs(detected_car_left, current_s, current_v, T_);
            costs.push_back(current_cost);
        }
        else if(states[i].compare("LCR")==0){
            double current_cost=total_costs(detected_car_right, current_s, current_v, T_);
            costs.push_back(current_cost);
        }
        else if(states[i].compare("KL")==0){
            double current_cost=total_costs(detected_car_middle, current_s, current_v, T_);
            costs.push_back(current_cost);
        }
        
    }
    //double best_cost;
    string best_state; 
    vector<double>::iterator best_cost=std::min_element(begin(costs), end(costs));
    int best_idx=std::distance(begin(costs), best_cost);
    best_state=states[best_idx];

    return lane_direction[best_state];
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



vector<Vehicle> Vehicle::const_speed(double duration){
    vector<Vehicle> trajectory;
    trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state));
    double new_pos=this->s+this->v*duration+0.5*this->a*duration*duration;
    trajectory.push_back(Vehicle(this->lane, new_pos, this->d, this->v, 0, this->state));
    return trajectory;
}

void Vehicle::lane_keep_trajectory(vector<vector<double>> detected_car_list, double &v_init, double &v_end, double &car_v_init, double &car_v_end, double duration, bool &flag){
    vector<Vehicle> trajectory;
    //vector<double> kinematic=get_kinematic(predictions, this->lane, duration);
    if(abs(v_end-current_v)<0.05&&flag==false){
        flag=true;
        car_v_init=current_v;
        car_v_end=current_v;
        v_init=car_v_init;
        v_end=car_v_end;
    }
    int idx=-1;
    int detected_car_size=detected_car_list.size();
    for(auto i=0; i<detected_car_list.size(); ++i){
        if(detected_car_list[detected_car_size-i-1][1]>current_s&&detected_car_list[detected_car_size-i-1][1]<(this->s+DIST_BUFFER)){
            double tar_car_s=detected_car_list[detected_car_size-i-1][1];
            double tar_car_v=detected_car_list[detected_car_size-i-1][2];
            double expected_v_end=current_v+2*duration;
            if(idx==-1){
                idx=detected_car_size-i-1;
                if(flag==true){
                    car_v_init=current_v;
                    car_v_end=std::min(std::min(MAX_SPEED, tar_car_v-1),expected_v_end);
                    v_init=car_v_init;
                    v_end=car_v_end;
                    flag=false;
                    //s_history.erase( s_history.begin()+10, s_history.end() );
                    //d_history.erase( d_history.begin()+10, d_history.end() );
                }
            }
        }
    }
    if(idx==-1&&flag==true&&v_end<(MAX_SPEED-1.0)){
        flag=false;
        car_v_init=current_v;
        car_v_end=MAX_SPEED;
        v_init=car_v_init;
        v_end=car_v_end;
        //s_history.erase( s_history.begin()+10, s_history.end() );
        //d_history.erase( d_history.begin()+10, d_history.end() );
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->d, v_init, this->a, this->state));

}
 
void Vehicle::lane_change_trajectory(double &v_init, double &v_end, 
                                    double &car_v_init, double &car_v_end, bool &flag, double &d_init, 
                                    double &d_end, double &car_d_init, double &car_d_end, double lane_direction){
    //vector<Vehicle> trajectory;
    //trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    //int new_lane;
    //new_lane=this->lane+lane_direction[state];
    //Vehicle next_lane_vehicle;
   //double new_d= 2.0+4.0*new_lane;
    //vector<double> new_kinematic=get_kinematic(predictions, new_lane, duration);
    //trajectory.push_back(Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state));
    //trajectory.push_back(Vehicle(new_lane, new_kinematic[0], new_d, new_kinematic[1], new_kinematic[2], state));
    //return trajectory;
    if (flag == true){
      if (lane_direction < 0){
        //cout << setw(25) << "Change d to: " << d_end - car_lane_width  << endl;
        car_d_init = current_d;
        if (current_d > 8.0){car_d_end = car_d_init-0.05;}
        else {car_d_end = car_d_init- 4.0+0.35;}
      }
      else{
        //cout << setw(25) << "Change d to: " << d_end + car_lane_width  << endl;
        car_d_init = current_d;
        if (current_d > 1.0*4.0){car_d_end = car_d_init + 4.0 - 0.75;}
        else {car_d_end = car_d_init-0.05;}
      }
      flag = false;
      d_init = car_d_init;
      d_end = car_d_end;
      if (s_history.size()>10){
        //s_history.erase( s_history.begin()+10, s_history.end() );
        //d_history.erase( d_history.begin()+10, d_history.end() );
      }
    }

  if ( abs(current_d-car_d_end) < 0.01*4.0 && flag == false) {
       flag = true;
       //car_d_init_global = car_d;
       //car_d_end_global = car_d;
       //d_init = car_v_init_global;
       //d_end = car_v_end_global;
  }
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



