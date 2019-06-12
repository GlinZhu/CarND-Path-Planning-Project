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
double max_s=6945.554;
double lane_width=4.0;
double car_s_init=1.249392e+02;
double car_d_init=6.164833e+00;

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
  //int lane=1;
  //create a class
  Vehicle ego_vehicle=Vehicle();

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_vehicle, &ref_v]
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          //cout<<"the size of sensor fusion is :"<<sensor_fusion.size()<<endl;
          //cout<<"sensor fusion id:"<<sensor_fusion[4][0]<<" x:"<<sensor_fusion[4][1]<<" y:"<<sensor_fusion[4][2]<<" vx:"<<sensor_fusion[4][3]<<" vy:"<<sensor_fusion[4][4]<<" s:"<<sensor_fusion[4][5]<<" d:"<<sensor_fusion[4][6]<<endl;
          //cout<<"sensor fusion id:"<<sensor_fusion[0][6]<<" x:"<<sensor_fusion[2][6]<<" y:"<<sensor_fusion[4][6]<<" vx:"<<sensor_fusion[6][6]<<" vy:"<<sensor_fusion[8][6]<<" s:"<<sensor_fusion[10][6]<<" d:"<<sensor_fusion[11][6]<<endl;
          
          int pre_size=previous_path_x.size();
          //cout<<"===============start the code==============================="<<endl;
         // cout<<"the size of previous path: "<<pre_size<<endl;
          int avail_path=std::min(MAX_PATH_KEPT, pre_size);
          //cout<<"the size of avail path: "<<avail_path<<endl;
          json msgJson;
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //std::cout<<"the data type of sensor fusion"<<typeid(sensor_fusion).name()<<"\n";
          /**
           * define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //Get lane value according to given current car_d;
          

// detect other vehciles around
          bool car_left, car_ahead, car_right;
          bool LaneChange=false, TooClose=false;
          vector<bool> car_detected;
          if(pre_size>0){
            car_s=end_path_s;
            car_d=end_path_d;
          }

          int ego_lane;
          ego_lane=LaneDetect(car_d);
          for(auto i=0;i<sensor_fusion.size();++i){
            double otherCar_d=sensor_fusion[i][6];
            double r_s=sensor_fusion[i][5];
            double r_vx=sensor_fusion[i][3];
            double r_vy=sensor_fusion[i][4];
            double r_v=sqrt(r_vx*r_vx+r_vy*r_vy);
            r_s+=(double)pre_size*TIMESTEP*r_v;
            double d_diff=otherCar_d-car_d;
            //cout<<"the predicted vehicles s is :"<<r_s<<endl;
            //cout<<"the difference between predicted vehicles and ego vehicle:"<<r_s-car_s<<endl;
            if(d_diff>2&&d_diff<6&&(fabs(car_s-r_s))<SAFETYGAP){
                car_right=true;
                cout<<"Car Right, Vid is !"<<sensor_fusion[i][0]<<endl;
            }
            else if(d_diff>-6&&d_diff<-2&&(fabs(car_s-r_s))<SAFETYGAP){
                car_left=true;
                cout<<"Car on the left!"<<endl;
            }
            else if(d_diff>-2&&d_diff<2){
                double vx=sensor_fusion[i][3];
                double vy=sensor_fusion[i][4];
                double FrontCar_v=sqrt(vx*vx+vy*vy);
                double FrontCar_s=sensor_fusion[i][5];
                FrontCar_s+=(double)pre_size*TIMESTEP*FrontCar_v;
                if((FrontCar_s>car_s)&&((FrontCar_s-car_s)<DIST_BUFFER)){
                  TooClose=true;
                  cout<<"the vehicle is too close, ready to change lane"<<endl;
                  car_ahead=true;
                  if(ego_lane>0){
                    ego_lane=ego_lane-1;
                    cout<<"change to the left lane"<<endl;
                  }
                  
                }
            }

          }
          

          if(TooClose){
            ref_v-=0.3;
            LaneChange=true;
            std::cout<<"ready to change the lane"<<endl;
          }
          else if(ref_v<21.5){
            ref_v+=0.4;
          }
          
          









/* ============================Trajectory generation ==========================================*/
	        vector<double> ptsx;
          vector<double> ptsy;
          double ref_x, ref_y, ref_x_pre, ref_y_pre;
          double pre_s, prev_d;
          ref_x=car_x;
          ref_y=car_y;
          //pre_s=s_pos-car_V*TIMESTEP;
          vector<double> pt_s;
          double ref_yaw=deg2rad(car_yaw);
          //cout<<"the size of previous path: "<<pre_size<<endl;

          if(pre_size<2){
            ref_x_pre=car_x-cos(ref_yaw);
            ref_y_pre=car_y-sin(ref_yaw);
            ptsx.push_back(ref_x_pre);
            ptsx.push_back(car_x);
            ptsy.push_back(ref_y_pre);
            ptsy.push_back(car_y);
            //double pre_s=s_pos-1.0;
            //pt_s.push_back(pre_s);
            //pt_s.push_back(s_pos);
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
            //pt_s.push_back(pre_s);
            //pt_s.push_back(s_pos);
          }
          //cout<<"tets point 1"<<endl;
          //for(unsigned i=0;i<pt_s.size();++i){
          //  cout<<"The current pt_s is "<<pt_s[i]<<endl;
          //}
          
          //std::cout << std::setw(25) << "=======================Trajectory Generation ===============================" << std::endl;
          
          
          //creating a path
          //shifting to the vehicle coordinates
          //creating waypoints of a spline with 30m space evenly
          vector<double> WP1;
          vector<double> WP2;
          vector<double> WP3;
          double d_target=2.0+4.0*ego_lane;
          double current_s_pos=car_s;
          double s_next=current_s_pos+30;
          double s_next1=current_s_pos+60;
          double s_next2=current_s_pos+90;
          WP1=getXY(s_next, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          WP2=getXY(s_next1, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          WP3=getXY(s_next2, d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(WP1[0]);
          ptsx.push_back(WP2[0]);
          ptsx.push_back(WP3[0]);
          //pt_s.push_back(s_next);
          //pt_s.push_back(s_next1);
          //pt_s.push_back(s_next2);
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

          
          
          // generate a trajectory using s_trajectory and pts_x as x, and y axis, and interpolated using increment_s_points
          if(ptsy.size()!=ptsx.size()){
            std::cout<<"error, there is mismatch between pt_s and ptsx"<<std::endl;
          }
          
          for(auto i=0;i<pre_size;++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          //debug
          /*
          for(unsigned i=0;i<pre_size;++i){
            cout<<"The previous path points are "<<previous_path_x[i]<<endl;
          }
          */

    


          for(auto i=0;i<ptsx.size();++i){
            double shifted_x=ptsx[i]-ref_x;
            double shifted_y=ptsy[i]-ref_y;
            ptsx[i]=cos(ref_yaw)*shifted_x+sin(ref_yaw)*shifted_y;
            ptsy[i]=sin(-ref_yaw)*shifted_x+cos(ref_yaw)*shifted_y;
          }
          tk::spline t;
          t.set_points(ptsx,ptsy);
  
          double target_x=30;
          double target_y;
          target_y=t(target_x);
          double tar_dist=distance(0, 0, target_x, target_y);
          double x_add_on(0);
          for(auto i=1;i<=50-pre_size;++i){
            double N=tar_dist/(ref_v*TIMESTEP);
            double x_point=x_add_on+target_x/N;
            x_add_on=x_point;
            double y_point=t(x_point);
            double shifted_x=x_point;
            double shifted_y=y_point;
            double new_x=cos(ref_yaw)*shifted_x-sin(ref_yaw)*shifted_y;
            double new_y=sin(ref_yaw)*shifted_x+cos(ref_yaw)*shifted_y;
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

