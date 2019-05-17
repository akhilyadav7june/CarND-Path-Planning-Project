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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using namespace std;

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

   // Default start at middle lane.
  int lane = 1;
  // Reference velocity.
  double ref_vel = 20.0; 
    
  h.onMessage([&lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          auto sensor_fusion = j[1]["sensor_fusion"];

           
          // Provided previous path point size.
          int prev_size = previous_path_x.size();

          // Preventing collitions.
          if (prev_size > 0) {
              car_s = end_path_s;
          }
                       
          // Prediction
          //Analysing nearest car positions 
            bool is_car_ahead = false;
            bool is_car_left = false;
            bool is_car_right = false;
            //double car_ahead_speed = ref_vel;
            
            for ( int i = 0; i < sensor_fusion.size(); i++ ) {
                float d = sensor_fusion[i][6];
                int car_lane = -1;
                                
                // is it on the same lane we are
                if ( d > 0 && d < 4 ) {
                  car_lane = 0;
                } else if ( d > 4 && d < 8 ) {
                  car_lane = 1;
                } else if ( d > 8 && d < 12 ) {
                  car_lane = 2;
                }
                
                if (car_lane < 0) {
                  continue;
                }
                
                // Finding car speed.
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
                
                // Estimating car_s position after executing previous trajectory.
                check_car_s += ((double)prev_size*0.02*check_speed);

                if ( car_lane == lane && check_car_s > car_s && check_car_s - car_s < 30) {
                  is_car_ahead = true;
                  //car_ahead_speed = check_speed;
                } else if ( car_lane - lane == -1 && car_s - 30 < check_car_s && car_s + 30 > check_car_s) {
                  is_car_left = true;
                } else if ( car_lane - lane == 1 && car_s - 30 < check_car_s && car_s + 30 > check_car_s) {
                  is_car_right = true;
                }
            }
          
          // Behavior
          //Performing action based on prediction
            
            const double MAX_SPEED = 49.5;
            const double MAX_ACC = .224;
                          
            if ( is_car_ahead ) {
                            
              if ( !is_car_left && lane > 0 ) {
                //Chaging left lane if there is no car in the left lane
                  lane--; 
              
              } else if ( !is_car_right && lane != 2 ){
                //Changing right lane if there is no car in the right lane
                  lane++; 
                  
              } else {
            
                  ref_vel -= MAX_ACC;
              }
             }else if ( ref_vel < MAX_SPEED ) {
                ref_vel += MAX_ACC;
            }
            
            //creating a list of widely spaced waypoints
            //later we will interpolate these waypoints with a spline library 
            vector<double> pts_x;
            vector<double> pts_y;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // Using car as starting reference if previous size is empty
            if ( prev_size < 2 ) {
                
                //Using two points that make the path tangent to the car
                
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                pts_x.push_back(prev_car_x);
                pts_x.push_back(car_x);

                pts_y.push_back(prev_car_y);
                pts_y.push_back(car_y);
                
            } else {
                //Using the previous path end's point as starting refernce 
                
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                pts_x.push_back(ref_x_prev);
                pts_x.push_back(ref_x);

                pts_y.push_back(ref_y_prev);
                pts_y.push_back(ref_y);
            }

            // Setting up target points in the future.
            vector<double> next_waypoint0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_waypoint1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_waypoint2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            pts_x.push_back(next_waypoint0[0]);
            pts_x.push_back(next_waypoint1[0]);
            pts_x.push_back(next_waypoint2[0]);

            pts_y.push_back(next_waypoint0[1]);
            pts_y.push_back(next_waypoint1[1]);
            pts_y.push_back(next_waypoint2[1]);

            
            for ( int i = 0; i < pts_x.size(); i++ ) {
              
              //Shifting car reference angle to 0 deg.
              double shift_x = pts_x[i] - ref_x;
              double shift_y = pts_y[i] - ref_y;

              pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            // Creating spline.
            tk::spline s;
            
            //setting x,y points to the spline
            s.set_points(pts_x, pts_y);

            //Actual x,y points we will use for planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            //starting with all of the previous path points from last time
            for ( int i = 0; i < previous_path_x.size(); i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            //Breaking up spline points so that we can travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;

            for( int i = 1; i < 50 - previous_path_x.size(); i++ ) {
              
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;
              
              //coverting back to global coordinate
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }         
         
          json msgJson;

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