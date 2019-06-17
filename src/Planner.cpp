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

double Vehicle::next_chosen_states(vector<vector<double>> detected_car_left, vector<vector<double>> detected_car_middle, vector<vector<double>> detected_car_right, double d_init, int pre_size){
    vector<string> states;
    states.push_back("KL");
    //string state=this->state;
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
    //debug
    for(unsigned i=0; i<states.size(); ++i){
        cout<<"the available states are: "<<states[i]<<" "<<endl;;
    }
    //cout<<"the available states are: "<<states[0]<<" "<<states[1]<<" "<<states[2]<<endl;;
    vector<double> costs;
    for(auto i=0; i<states.size(); ++i){
        //std::cout<<"start costs loop: "<<i<<endl;
        if(states[i].compare("KL")==0){
            if(current_d>8.0){
                double d_end=this->current_d;
                double current_cost=total_costs(detected_car_right, this->current_s, this->s, this->current_v, d_init, d_end,2, pre_size);
                costs.push_back(current_cost);
            }
            else if(current_d>4.0){
                double d_end=this->current_d;
                double current_cost=total_costs(detected_car_middle, this->current_s, this->s,this->current_v, d_init, d_end,2, pre_size);
                costs.push_back(current_cost);
            }
            else{
                double d_end=this->current_d;
                double current_cost=total_costs(detected_car_left, this->current_s, this->s,this->current_v, d_init, d_end,2, pre_size);
                costs.push_back(current_cost);
            }
            
        }
        else if(states[i].compare("LCL")==0){
            if(current_d>8.0){
                double d_end=this->current_d-4.0;
                double current_cost=total_costs(detected_car_middle, this->current_s, this->s,this->current_v, d_init, d_end, 2, pre_size);
                costs.push_back(current_cost);                
            }
            else if(current_d>4.0){
                double d_end=this->current_d-4.0;
                double current_cost=total_costs(detected_car_left, this->current_s, this->s,this->current_v, d_init, d_end, 2, pre_size);
                costs.push_back(current_cost);   
            }
            
        }
        else if(states[i].compare("LCR")==0){
            if(current_d>4.0&&current_d<8){
                double d_end=this->current_d+4.0;
                double current_cost=total_costs(detected_car_right, this->current_s, this->s,this->current_v, d_init, d_end,2, pre_size);
                costs.push_back(current_cost);
            }
            else if(current_d<4){
                double d_end=this->current_d+4.0;
                double current_cost=total_costs(detected_car_middle, this->current_s, this->s,this->current_v, d_init, d_end,2, pre_size);
                costs.push_back(current_cost);
            }
            
        }
        
        //std::cout<<"the costs loop is completed: "<<endl;
        
    }
    //double best_cost;
    //debug
    for(unsigned i=0; i<costs.size(); ++i){
        cout<<"the costs are: "<<costs[i]<<" "<<endl;;
    }
    string best_state; 
    vector<double>::iterator best_cost=std::min_element(begin(costs), end(costs));
    int best_idx=std::distance(begin(costs), best_cost);
    best_state=states[best_idx];
    std::cout<<"the best state is: "<<best_state<<std::endl;
    std::cout<<"the lane direction is: "<<lane_direction[best_state]<<std::endl;
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


/* 
void Vehicle::lane_keep_trajectory(vector<vector<double>> detected_car_list, double &v_init, double &v_end, double &car_v_init, 
                double &car_v_end, double duration, bool &flag, vector<double> &s_history, vector<double> &d_history){
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
            double tar_car_v=detected_car_list[detected_car_size-i-1][3];
            //double expected_v_end=current_v+2*duration;
            if(idx==-1){
                idx=detected_car_size-i-1;
                if(flag==true){
                    car_v_init=current_v;
                    car_v_end=(std::min(MAX_SPEED, tar_car_v-1));
                    v_init=car_v_init;
                    v_end=car_v_end;
                    flag=false;
                    s_history.erase( s_history.begin()+10, s_history.end() );
                    d_history.erase( d_history.begin()+10, d_history.end() );
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
        s_history.erase( s_history.begin()+10, s_history.end() );
        d_history.erase( d_history.begin()+10, d_history.end() );
    }
    //trajectory.push_back(Vehicle(this->lane, this->s, this->d, v_init, this->a, this->state));

}
*/
void Vehicle::lane_keep_trajectory(vector<vector<double>>  sensor_car_list_current,
   double &v_init, double &v_end, double &car_v_init_global, double &car_v_end_global, bool &flag, vector<double> &s_history, vector<double> &d_history){
    double car_speed=this->current_v;
    double car_s=this->current_s;
    double prev_s=this->s;
    double lane_keeping_buffer=5;
    double lane_keeping_buffer_v=1.0;
    double car_speed_max=MAX_SPEED;
  if ( abs(car_speed-v_end) < 0.5 && flag == false) {
       //cout << setw(25) << "Target speed reached: " << car_speed << endl;
       flag = true;
       car_v_init_global = car_speed;
       car_v_end_global = car_speed;
       v_init = car_v_init_global;
       v_end = car_v_end_global;
  }

  double idx = -1;

  for(int i=0; i<sensor_car_list_current.size(); i++){

    //if(dflag>=dflag_sensor_details){cout << "All cars in the CURRENT lane: " << sensor_car_list_current[sensor_car_list_current.size()-i-1][1]-car_s << endl;}

    if(sensor_car_list_current[sensor_car_list_current.size()-i-1][1]>car_s){

      cout << "Cars in front: " << sensor_car_list_current[sensor_car_list_current.size()-i-1][1]-car_s << endl;

      if (sensor_car_list_current[sensor_car_list_current.size()-i-1][1]<prev_s+ lane_keeping_buffer){

        double car_v_target = sensor_car_list_current[sensor_car_list_current.size()-i-1][3];
        double car_s_target = sensor_car_list_current[sensor_car_list_current.size()-i-1][1];

        //cout << car_v_target << " " << ((car_v_target-v_end)< 0.5*lane_keeping_buffer_v) << " " << (car_v_target < car_speed_max) << " " << flag << endl;

        if (idx == -1) {

          idx = sensor_car_list_current.size()-i-1;
          if ( (flag == true) && (car_s_target-car_s<2.0*lane_keeping_buffer) ){
            //cout << setw(25) << "Attension: Car too close "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            //cout << setw(25) << "Decrease speed to: " << car_v_target - lane_keeping_buffer_v*3.0  << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_v_target - lane_keeping_buffer_v*3.0;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
          else if (flag == true && (car_v_target>car_speed_max) && (v_end<car_speed_max)) {
            //cout << setw(25) << "Attension: "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            //cout << setw(25) << "Increase speed to: " << car_speed_max  << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_speed_max;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
          else if(  (((flag==false)&&(v_end==car_speed_max))||(flag==true)) && (car_v_target<car_speed_max)&&((car_v_target-v_end)<0.5*lane_keeping_buffer_v) )
          {
            //cout << setw(25) << "Attension: "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            //cout << setw(25) << "Decrease speed to: " << car_v_target-lane_keeping_buffer_v << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_v_target - lane_keeping_buffer_v;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
          else if(flag == true && car_v_target<car_speed_max && car_v_target-v_end > 1.5*lane_keeping_buffer_v){
            //cout << setw(25) << "Attension: "; for (int j=0; j<4; j++) { cout << sensor_car_list_current[sensor_car_list_current.size()-i-1][j] << " ";} cout << endl;
            //cout << setw(25) << "Increase speed to: " << car_v_target-lane_keeping_buffer_v << endl;
            flag = false;
            car_v_init_global = car_speed;
            car_v_end_global = car_v_target - lane_keeping_buffer_v;
            v_init = car_v_init_global;
            v_end = car_v_end_global;
            s_history.erase( s_history.begin()+10, s_history.end() );
            d_history.erase( d_history.begin()+10, d_history.end() );
          }
        }

      }
    }
  }

  if (flag == true && idx == -1 && v_end < car_speed_max - lane_keeping_buffer_v){
    //cout << setw(25) << "Attension: detected NO car in front"  << endl;
    //cout << setw(25) << "Increase speed to: " << car_speed_max  << endl;
    flag = false;
    car_v_init_global = car_speed;
    car_v_end_global = car_speed_max;
    v_init = car_v_init_global;
    v_end = car_v_end_global;
    s_history.erase( s_history.begin()+10, s_history.end() );
    d_history.erase( d_history.begin()+10, d_history.end() );
  }
}

 
void Vehicle::lane_change_trajectory(double &v_init, double &v_end, bool &flag, double &d_init, 
                                    double &d_end, double &car_d_init_global, double &car_d_end_global, double lane_direction, vector<double> &s_history, vector<double> &d_history){
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
        car_d_init_global = current_d;
        if (current_d > 8.0){car_d_end_global = 6.0-0.05;}
        else {car_d_end_global = 6.0- 4.0+0.35;}
      }
      else{
        //cout << setw(25) << "Change d to: " << d_end + car_lane_width  << endl;
        car_d_init_global = current_d;
        if (current_d > 1.0*4.0){car_d_end_global = 6.0 + 4.0 - 0.75;}
        else {car_d_end_global = 6.0-0.05;}
      }
      flag = false;
      d_init = car_d_init_global;
      d_end = car_d_end_global;
      if (s_history.size()>10){
        s_history.erase( s_history.begin()+10, s_history.end() );
        d_history.erase( d_history.begin()+10, d_history.end() );
      }
    }

  if(abs(current_d-car_d_end_global) < 0.01*4.0 && flag == false) {
       flag = true;
       //car_d_init_global = car_d;
       //car_d_end_global = car_d;
       //d_init = car_v_init_global;
       //d_end = car_v_end_global;
  }
}


/* 
bool Vehicle::get_vehicle_ahead(vector<vector<double>> detected_car_middle, double pre_car_s, double pre_size){
    bool found=false;
    Vehicle temp_vehicle;
    int min_s=std::numeric_limits<int>::max();
    for(auto i=0;i<detected_car_middle.size();++i){
        //temp_vehicle=predictions[i];
        double Front_s=detected_car_middle[i][1];
        double Front_v=detected_car_middle[i][3];
        Front_s+=Front_v*TIMESTEP*pre_size;
        if((Front_s>pre_car_s)&&((Front_s-pre_car_s)<DIST_BUFFER)){
            found=true;
        }
        return found;
    }
    return found;
}
*/

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



