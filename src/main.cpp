#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <math.h>

#include "Planner.h" //include the planner functions
#include "Constant.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
//initialize variables

double lane_width=4.0;
double car_s_init=1.249392e+02;
double car_d_init=6.164833e+00;
double car_v_init=0.0;
double car_v_end=MAX_SPEED;
double car_d_end=car_d_init;
vector<double> s_history;
vector<double> d_history;
int max_path_points=100;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0


  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  tk::spline s_x, s_y, s_dx, s_dy;

  spline_fitting(s_x, s_y, s_dx, s_dy, map_waypoints_s,map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);



  //double ref_v=0.0;
  //int lane=1;
  //create a class
  Vehicle ego_vehicle=Vehicle();

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_vehicle, &s_x, &s_y, &s_dx, &s_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          car_speed=car_speed/2.23694; //covert mph to m/s

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          cout<<"the size of sensor fusion is :"<<sensor_fusion.size()<<endl;
          cout<<"sensor fusion id:"<<sensor_fusion[4][0]<<" x:"<<sensor_fusion[4][1]<<" y:"<<sensor_fusion[4][2]<<" vx:"<<sensor_fusion[4][3]<<" vy:"<<sensor_fusion[4][4]<<" s:"<<sensor_fusion[4][5]<<" d:"<<sensor_fusion[4][6]<<endl;
          //cout<<"sensor fusion id:"<<sensor_fusion[0][6]<<" x:"<<sensor_fusion[2][6]<<" y:"<<sensor_fusion[4][6]<<" vx:"<<sensor_fusion[6][6]<<" vy:"<<sensor_fusion[8][6]<<" s:"<<sensor_fusion[10][6]<<" d:"<<sensor_fusion[11][6]<<endl;
          
          int pre_size=previous_path_x.size();
          //cout<<"===============start the code==============================="<<endl;
          cout<<"the size of previous path: "<<pre_size<<endl;
          int avail_path=std::min(MAX_PATH_KEPT, pre_size);
          //cout<<"the size of avail path: "<<avail_path<<endl;
          json msgJson;
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          vector<vector<double>> detected_car_left;
          vector<vector<double>> detected_car_middle;
          vector<vector<double>> detected_car_right;
          vector<double> start, end;
          int sensor_fusion_size;
          double current_car_s, current_car_d, current_car_v;
          current_car_s=car_s;
          current_car_d=car_d;
          current_car_v=car_speed;
          //std::cout<<"the data type of sensor fusion"<<typeid(sensor_fusion).name()<<"\n";
          /**
           * define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //Get lane value according to given current car_d;
          int lane;
		      lane=LaneDetect(car_d);

          double pre_s_pos, pre_s_pos2, pre_d_pos, pre_d_pos2, pre_car_v, pre_car_v2, pre_d_dot, pre_d_dot2, pre_d_ddot, pre_car_accel, pre_car_angle;
          if(pre_size<2){
            pre_car_angle=deg2rad(car_yaw);
            pre_s_pos=car_s_init;
            pre_d_pos=car_d_init;
            pre_car_v=car_v_init;
            pre_d_dot=0.0;
            pre_d_ddot=0.0;
            pre_car_accel=0.0;

          }
          else{
            //assign the current state to Vehicle variables
            pre_s_pos=s_history[s_history.size()-1];
            pre_d_pos=d_history[d_history.size()-1];
            pre_s_pos2=s_history[s_history.size()-2];
            pre_car_v=(pre_s_pos-pre_s_pos2)/TIMESTEP;
            pre_car_v2=(s_history[s_history.size()-2]-s_history[s_history.size()-3])/TIMESTEP;
            pre_car_accel=(pre_car_v-pre_car_v2)/TIMESTEP;
            pre_d_pos=d_history[d_history.size()-1];
            pre_d_pos2=d_history[d_history.size()-2];
            pre_d_dot=(pre_d_pos-pre_d_pos2)/TIMESTEP;
            pre_d_dot2=(d_history[d_history.size()-2]-d_history[d_history.size()-3])/TIMESTEP;
            pre_d_ddot=(pre_d_dot-pre_d_dot2)/TIMESTEP;
            //debug
            std::cout<<"  pre_s_pos: "<<pre_s_pos<<"  pre_d_pos: "<<pre_d_pos<<" pre_car_v: "<<pre_car_v<<" pre_car_accel: "<<pre_car_accel<<std::endl;
            // erase passed path point to correct current s_history
            s_history.erase(s_history.begin(), s_history.begin()+(max_path_points-pre_size));
            d_history.erase(d_history.begin(), d_history.begin()+(max_path_points-pre_size));
          }
//========initialize the vehicle state
          ego_vehicle.lane=lane;
          ego_vehicle.s=pre_s_pos;
          ego_vehicle.d=pre_d_pos;
          ego_vehicle.v=pre_car_v;
          ego_vehicle.a=pre_car_accel;
          ego_vehicle.d_dot=pre_d_dot;
          ego_vehicle.d_ddot=pre_d_ddot;
          //ego_vehicle.state="CS";
          ego_vehicle.target_v=MAX_SPEED;
          // Pre-process the sensor fusion
          sensor_processing(sensor_fusion, detected_car_left, detected_car_middle, detected_car_right);
          // Implement the behavior planner
          











/* ======================Start the iterations===============================*/
          std::cout<< "=======Start the iterations============"<<std::endl;
          double duration=T_-avail_path*TIMESTEP;
          //vector<vector<double>> Best_traj;
          vector<vector<double>> Best_target;

          Best_target=ego_vehicle.get_best_traj(sensor_fusion, duration); //which contains {s_target, d_target}
          std::cout<<"Best target of s in main :"<<Best_target[0][0]<<" "<<Best_target[0][1]<<" "<<Best_target[0][2]<<std::endl;
          std::cout<<"Best target of d in main :"<<Best_target[1][0]<<" "<<Best_target[1][1]<<" "<<Best_target[1][2]<<std::endl;
    
          //std::cout<<"Best target state: "<<Best_target[0][1]<<std::endl;






/* ============================Trajectory generation ==========================================*/
	        vector<double> ptsx;
          vector<double> ptsy;
          double ref_x, ref_y, ref_x_pre, ref_y_pre;
          double pre_s, prev_d;
          ref_x=car_x;
          ref_y=car_y;
          pre_s=s_pos-car_V*TIMESTEP;
          vector<double> pt_s;
          double ref_yaw=deg2rad(car_yaw);
          //cout<<"the size of previous path: "<<pre_size<<endl;

          if(avail_path<2){
            ref_x_pre=car_x-cos(ref_yaw);
            ref_y_pre=car_y-sin(ref_yaw);
            ptsx.push_back(ref_x_pre);
            ptsx.push_back(car_x);
            ptsy.push_back(ref_y_pre);
            ptsy.push_back(car_y);
            double pre_s=s_pos-1.0;
            pt_s.push_back(pre_s);
            pt_s.push_back(s_pos);
          }
          else{
            ref_x=previous_path_x[avail_path-1];
            ref_x_pre=previous_path_x[avail_path-2];
            ref_y=previous_path_y[avail_path-1];
            ref_y_pre=previous_path_y[avail_path-2];
            ref_yaw=atan2(ref_y-ref_y_pre, ref_x-ref_x_pre);
            ptsx.push_back(ref_x_pre);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_pre);
            ptsy.push_back(ref_y);
            pt_s.push_back(pre_s);
            pt_s.push_back(s_pos);
          }
          //cout<<"tets point 1"<<endl;
          //for(unsigned i=0;i<pt_s.size();++i){
          //  cout<<"The current pt_s is "<<pt_s[i]<<endl;
          //}
          
          std::cout << std::setw(25) << "=======================Trajectory Generation ===============================" << std::endl;
          
          
          //creating a path
          //shifting to the vehicle coordinates
          //creating waypoints of a spline with 30m space evenly
          vector<double> WP1;
          vector<double> WP2;
          vector<double> WP3;
          double d_target=Best_target[1][0];
          double s_next=s_pos+30;
          double s_next1=s_pos+60;
          double s_next2=s_pos+90;
          WP1=getXY(s_next, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          WP2=getXY(s_next1, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          WP3=getXY(s_next2, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(WP1[0]);
          ptsx.push_back(WP2[0]);
          ptsx.push_back(WP3[0]);
          pt_s.push_back(s_next);
          pt_s.push_back(s_next1);
          pt_s.push_back(s_next2);
          ptsy.push_back(WP1[1]);
          ptsy.push_back(WP2[1]);
          ptsy.push_back(WP3[1]);

          //debug ================
          //for(unsigned i=0;i<pt_s.size();++i){
          //  cout<<"The current pt_s is "<<pt_s[i]<<endl;
          //}
          //for(unsigned i=0;i<ptsy.size();++i){
          //  cout<<"The current ptsy is "<<ptsy[i]<<endl;
          //}

          //=========================================

          double max_v_incre=MAX_ACCEL*TIMESTEP-0.03; // 
          vector<double> increment_s_points;
          double v_incre;
          double s_dot_target=Best_target[0][1];
          double current_v=car_V;
          double current_s=s_pos;
          for(int i=0; i<50-avail_path; ++i){
            v_incre=(s_dot_target-current_v)/(fabs(s_dot_target-current_v))*max_v_incre;
            current_v+=v_incre;
            current_s+=current_v*TIMESTEP;
            increment_s_points.push_back(current_s);    //generate incremented s_points
            //cout<<"current v is: "<<current_v<<endl;
          }
          //cout<<"==========check inteploated points s: "<<endl;
          //cout<<"the incremented v is :"<<v_incre<<endl;

          //for(unsigned i=0;i<increment_s_points.size();++i){
          //  cout<<"The current increment_s_points is "<<increment_s_points[i]<<endl;
          //}
          
          // generate a trajectory using s_trajectory and pts_x as x, and y axis, and interpolated using increment_s_points
          if(pt_s.size()!=ptsx.size()){
            std::cout<<"error, there is mismatch between pt_s and ptsx"<<std::endl;
          }
          






          
          for(auto i=0;i<avail_path;++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          //debug
          /*
          for(unsigned i=0;i<pre_size;++i){
            cout<<"The previous path points are "<<previous_path_x[i]<<endl;
          }
          */

    
          //pt_s.erase(pt_s.begin());
          //ptsx.erase(ptsx.begin());
          //ptsy.erase(ptsy.begin());
          tk::spline t;
          t.set_points(pt_s, ptsx);
          
          cout<<"tets point 2"<<endl;
          for(unsigned i=0; i<increment_s_points.size(); ++i){
            double Next_x=t(increment_s_points[i]);
            next_x_vals.push_back(Next_x);
          }
          cout<<"next x val : "<<next_x_vals[avail_path+1]<<endl;
          // generate s splie for y axis;
          tk::spline s;
          s.set_points(pt_s, ptsy);
          for(unsigned i=0; i<increment_s_points.size(); ++i){
            double Next_y=s(increment_s_points[i]);
            next_y_vals.push_back(Next_y);
          }
          cout<<"next y val : "<<next_y_vals[avail_path+1]<<endl;
          
          cout<<"===========================end the code============================="<<endl;
        

/*
          for(auto i=0;i<ptsx.size();++i){
            double shifted_x=ptsx[i]-ref_x;
            double shifted_y=ptsy[i]-ref_y;
            ptsx[i]=cos(ref_yaw)*shifted_x+sin(ref_yaw)*shifted_y;
            ptsy[i]=sin(-ref_yaw)*shifted_x+cos(ref_yaw)*shifted_y;
          }
          tk::spline t;
          t.set_points(ptsx,ptsy);

*/
         

          //double target_x=30;
          //double target_y;
          //target_y=t(target_x);
          //double tar_dist=distance(0, 0, target_x, target_y);
    
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}

