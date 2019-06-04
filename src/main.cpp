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
//#include "Planner.cpp" // shouldn't include source file directly to avoid "undefined reference to error"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  double max_s = 6945.554;

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
  double ref_v=49;
  int lane=1;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_v, &lane]
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
          double AccT=j[1]["AccT"];
          double AccN=j[1]["AccN"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          int pre_size=previous_path_x.size();

          json msgJson;
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //std::cout<<"the data type of sensor fusion"<<typeid(sensor_fusion).name()<<"\n";
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
	        vector<double> ptsx;
          vector<double> ptsy;
          double ref_x, ref_y, ref_x_pre, ref_y_pre;
          ref_x=car_x;
          ref_y=car_y;
          double ref_yaw=deg2rad(car_yaw);
          Vehicle ego_vehicle;
          double car_accel=sqrt(AccT*AccT+AccN*AccN);
          //define the initial vehicle state;
          ego_vehicle=Vehicle(lane, car_s, car_speed/3.6, car_accel, "CS");
          //get the final trajectory according the previous vehicle state and sensor fusion;
          vector<Vehicle> final_trajectory=ego_vehicle.choose_next_state(sensor_fusion);
          
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
          //creating a path
          //shifting to the vehicle coordinates
          //creating waypoints of a spline with 30m space evenly
          vector<double> WP1;
          vector<double> WP2;
          vector<double> WP3;
          WP1=getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          WP2=getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          WP3=getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          tk::spline t;
          ptsx.push_back(WP1[0]);
          ptsx.push_back(WP2[0]);
          ptsx.push_back(WP3[0]);
          ptsy.push_back(WP1[1]);
          ptsy.push_back(WP2[1]);
          ptsy.push_back(WP3[1]);
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

          double target_x=30;
          double target_y;
          target_y=t(target_x);
          double tar_dist=distance(0, 0, target_x, target_y);
          double x_add_on(0.0);
          double N;
          

          for(auto i=1;i<=50-previous_path_x.size();++i){
            N=tar_dist/(0.02*ref_v/2.24);
            double x_point=x_add_on+target_x/N;
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
