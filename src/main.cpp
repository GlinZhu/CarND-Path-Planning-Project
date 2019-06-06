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
#include "Traj_generation.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

//initialize variables
double max_s=6945.554;
double lane_width=4.0;
double car_s_init=1.249392e+02;
double car_d_init=6.164833e+00;
double car_speed_max=24.5; // m/s

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
  double ref_v=0.0;
  int lane=1;
  //create a class
  Vehicle ego_vehicle=Vehicle();

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_vehicle]
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          //double AccT=j[1]["AccT"];
          //double AccN=j[1]["AccN"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          int pre_size=previous_path_x.size();

          json msgJson;
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //std::cout<<"the data type of sensor fusion"<<typeid(sensor_fusion).name()<<"\n";
          /**
           * define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //Get lane value according to given current car_d;
          int lane;

		      if(car_d>0&&car_d<4){
			      lane=0; //left
			    }
		      else if(car_d>4&&car_d<8){
			      lane=1; //middle
			    }
          else if(car_d>8&&car_d<12){
			      lane=2; //right
            }
	        vector<double> ptsx;
          vector<double> ptsy;
          double ref_x, ref_y, ref_x_pre, ref_y_pre;
          //double prev_s, prev_d;
          vector<double> s_history, d_history;
          int s_len=s_history.size();
          double car_accel;
          s_history.push_back(car_s);
          d_history.push_back(car_d);
          if(s_history.size()<=2){
            car_accel=2;
          }
          else{
            double cur_v=(s_history[s_len-1]-s_history[s_len-2])/0.02;
            double pre_v=(s_history[s_len-2]-s_history[s_len-3])/0.02;
            car_accel=(cur_v-pre_v)/0.02;
          }
          ref_x=car_x;
          ref_y=car_y;
          double ref_yaw=deg2rad(car_yaw);
          if(pre_size<2){
            ref_x_pre=car_x-cos(ref_yaw);
            ref_y_pre=car_y-sin(ref_yaw);
            ptsx.push_back(ref_x_pre);
            ptsx.push_back(car_x);
            ptsy.push_back(ref_y_pre);
            ptsy.push_back(car_y);

          }
          else{
            ref_x=previous_path_x[pre_size-1];
            ref_x_pre=previous_path_x[pre_size-2];
            ref_y=previous_path_y[pre_size-1];
            ref_y_pre=previous_path_y[pre_size-2];
            ref_yaw=atan2(ref_y-ref_y_pre, ref_x-ref_x_pre);
            ptsx.push_back(ref_x_pre);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_pre);
            ptsy.push_back(ref_y);
          }
          
          std::cout << std::setw(25) << "==================================================================" << std::endl;
          //class state initialization 
          ego_vehicle.target_v=24.5;   //m/s
          ego_vehicle.max_accel=10.0;
          ego_vehicle.dist_buffer=10.0;
          //double car_accel=sqrt(AccT*AccT+AccN*AccN);
          //define the initial vehicle state;
          ego_vehicle.lane=lane;
          ego_vehicle.s=car_s;
          ego_vehicle.v=car_speed/2.24;
          ego_vehicle.a=car_accel;
          ego_vehicle.state="CS";
          ego_vehicle=Vehicle(lane, car_s, car_speed/2.24, car_accel, "CS");
          
          //get the final trajectory according to the previous vehicle state and sensor fusion;
          vector<Vehicle> final_trajectory=ego_vehicle.choose_next_state(sensor_fusion);

          Vehicle veh_init=final_trajectory[0]; //the starting state of vehicle
          Vehicle veh_final=final_trajectory[1]; // the goal state of vehicle

          
          //creating a path
          //shifting to the vehicle coordinates
          //creating waypoints of a spline with 30m space evenly
          vector<double> WP1;
          vector<double> WP2;
          vector<double> WP3;
          // calculate coefficents for s(t) and d(t);
          vector<double> start_s(3), end_s(3);
          vector<double> start_d(3), end_d(3);

          start_s={car_s, veh_init.v, veh_init.a};
          end_s={veh_final.s, veh_final.v, veh_final.a};
          start_d={car_d, 0.0, 0.0}; //assume both start and end states have d_dot and d_ddot being zero
          double d_final=2.0+4.0*veh_final.lane;
          end_d={d_final, 0.0, 0.0};

          // Get coefficients of PTG
          double T=1.0;
          vector<double> coeff_s=JMT(start_s, end_s, T);
          vector<double> coeff_d=JMT(start_d, end_d, T);
          //generate trajectory using both s(t) and d(t)
          vector<double> T_;
          vector<double> traj_s;
          for(double i=0; i<T; i+=0.02){
            double pt=coeff_s[0]+coeff_s[1]*i+coeff_s[2]*i*i+coeff_s[3]*pow(i,3)+coeff_s[4]*pow(i,4)+coeff_s[5]*pow(i,5);
            traj_s.push_back(pt);
            T_.push_back(i);
          }

          //generate d trajectory;
          vector<double> traj_d;
          for(double i=0; i<T; i+=0.02){
            double pt=coeff_d[0]+coeff_d[1]*i+coeff_d[2]*i*i+coeff_d[3]*pow(i,3)+coeff_d[4]*pow(i,4)+coeff_d[5]*pow(i,5);
            traj_d.push_back(pt);
          }
          vector<double> waypoints;
          for(unsigned i=0; i<traj_s.size(); ++i){
            vector<double> WP=getXY(traj_s[i], traj_d[i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(WP[0]);
            ptsy.push_back(WP[1]);
          }
          


          //WP1=getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //WP2=getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //WP3=getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          tk::spline t;
          //ptsx.push_back(WP1[0]);
          ///ptsx.push_back(WP2[0]);
          //ptsx.push_back(WP3[0]);
          //ptsy.push_back(WP1[1]);
          //ptsy.push_back(WP2[1]);
          //ptsy.push_back(WP3[1]);

          for(auto i=0;i<ptsx.size();++i){
            double shifted_x=ptsx[i]-ref_x;
            double shifted_y=ptsy[i]-ref_y;
            ptsx[i]=cos(ref_yaw)*shifted_x+sin(ref_yaw)*shifted_y;
            ptsy[i]=sin(-ref_yaw)*shifted_x+cos(ref_yaw)*shifted_y;
          }
          t.set_points(ptsx,ptsy);



          for(auto i=0;i<previous_path_x.size();++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //double target_x=30;
          //double target_y;
          //target_y=t(target_x);
          //double tar_dist=distance(0, 0, target_x, target_y);
          double x_add_on(0.0);
          //double N;
          double dist_inc;
          double max_inc=0.445;
     
          double ref_v=veh_final.v;
          for(auto i=1;i<=50-previous_path_x.size();++i){
            dist_inc=(0.02*ref_v/2.24);
            double x_point=x_add_on+dist_inc;
            double y_point=t(x_point);
            x_add_on=x_point;
            double new_x=x_point;
            double new_y=y_point;
            new_x=cos(ref_yaw)*new_x-sin(ref_yaw)*new_y;
            new_y=sin(ref_yaw)*new_x+cos(ref_yaw)*new_y;
            new_x+=ref_x;
            new_y+=ref_y;
            next_x_vals.push_back(new_x);
            next_y_vals.push_back(new_y);
          }


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

