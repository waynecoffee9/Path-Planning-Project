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
  const double TRACK_L = 6945.554;

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
  // initialize lane
  int lane = 1;
  // reference velocity
  double ref_vel = 0.0;
  double set_vel = 0.0;  

  const int SHORT_PATH_POINTS = 5;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &set_vel, &lane, &ref_vel, &TRACK_L,
			   &SHORT_PATH_POINTS]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
  

	// reference delta time between two points
	double ref_t = 0.02;
	
	// number of path planning points to set at a time.  Default is 50.  
	// if environment causes change of action (e.g. change in velocity and lane), reduce the previous
	// path points to 5, and add modified path planning points based on appropriate maneuver
	// for acceleration at the beginning, assign 300 points which represents 6 seconds.
	int num_path_p = 50;
	
	// default is false.  When this is flipped to true, previous path planning points are replaced with new ones.
	bool stat_change = false;
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

		  // reference velocity, default 21.9m/s = 49mph
		  ref_vel = 21.9;

		  for (int i=0;i<sensor_fusion.size();i++) {
			  vector<double> car = sensor_fusion[i];
			  if (car[5] > car_s && (car[5] - car_s) < 50) {
				  if (car[6] > lane*4 && car[6] < (lane+1)*4) {
					  double speed = sqrt(car[3]*car[3]+car[4]*car[4]);
					  if (speed < ref_vel) {
						  double del_dist = car[5] - car_s;
						  double del_speed = ref_vel - speed;
						  ref_vel = speed;
						  num_path_p = std::min(del_dist / del_speed / ref_t+SHORT_PATH_POINTS,200+SHORT_PATH_POINTS);
						  stat_change = true;
						  std::cout << ref_vel << std::endl;
						  std::cout << set_vel << std::endl;
					  }
				  }					  
			  }
		  }
		  
		  // reduce path planning points down
		  if (stat_change) {
			  if (previous_path_x.size() > 5) {
				  previous_path_x.erase(previous_path_x.begin()+5, ve.end());
				  previous_path_y.erase(previous_path_x.begin()+5, ve.end());
			  }
			  int prev_size = previous_path_x.size();
		  }
		  
		  vector<double> ptsx;
		  vector<double> ptsy;
		  
		  double ref_x = car_x;
		  double ref_y = car_y;
		  double ref_yaw = deg2rad(car_yaw);


          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  vector<double> v_spline_x;
		  vector<double> v_spline_y;
		  
		  if(prev_size < 2) {
			  double prev_car_x = car_x - cos(car_yaw);
			  double prev_car_y = car_y - sin(car_yaw);
			  
			  ptsx.push_back(prev_car_x);
			  ptsx.push_back(car_x);
			  
			  ptsy.push_back(prev_car_y);
			  ptsy.push_back(car_y);
			  
			  v_spline_y.push_back(0.0);
			  v_spline_y.push_back(0.0);
			  v_spline_x.push_back(-ref_t);
			  v_spline_x.push_back(0.0);
		  }
		  else {
			  ref_x = previous_path_x[prev_size-1];
			  ref_y = previous_path_y[prev_size-1];
			  
			  double ref_x_prev = previous_path_x[prev_size-2];
			  double ref_y_prev = previous_path_y[prev_size-2];
			  ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
			  
			  ptsx.push_back(ref_x_prev);
			  ptsx.push_back(ref_x);
			  
			  ptsy.push_back(ref_y_prev);
			  ptsy.push_back(ref_y);
			  
			  double vel_f = distance(ref_x,ref_y,ref_x_prev,ref_y_prev)/ref_t;
			  double vel_o = distance(ref_x_prev,ref_y_prev,previous_path_x[prev_size-3],previous_path_y[prev_size-3])/ref_t;
			  v_spline_y.push_back(vel_o);
			  v_spline_y.push_back(vel_f);
			  v_spline_x.push_back(-ref_t);
			  v_spline_x.push_back(0.0);
		  }

		  vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		  vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		  vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		  
		  ptsx.push_back(next_wp0[0]);
		  ptsx.push_back(next_wp1[0]);
		  ptsx.push_back(next_wp2[0]);
		  
		  ptsy.push_back(next_wp0[1]);
		  ptsy.push_back(next_wp1[1]);
		  ptsy.push_back(next_wp2[1]);
		  
		  

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
		  // coordinate transformation
		  for (int i = 0; i < ptsx.size(); i++) {
			  double shift_x = ptsx[i]-ref_x;
			  double shift_y = ptsy[i]-ref_y;
			  
			  ptsx[i] = (shift_x * cos(-ref_yaw)-shift_y * sin(-ref_yaw));
			  ptsy[i] = (shift_x * sin(-ref_yaw)+shift_y * cos(-ref_yaw));
			  
		  }
		  
		  tk::spline s;
		  // fill next x and y points with previous path points first
		  s.set_points(ptsx, ptsy);
		  for (int i = 0; i < previous_path_x.size(); ++i) {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
		  }
		  
		  // generate spline for velocity increase/decrease over time
		  tk::spline v_spline;
		  double t_acc = std::max((ref_vel-v_spline_y.back())/3.65,0.2);
		  v_spline_y.push_back(ref_vel);
		  v_spline_y.push_back(ref_vel);
		  v_spline_x.push_back(t_acc);
		  v_spline_x.push_back(t_acc+ref_t);
		  v_spline.set_points(v_spline_x, v_spline_y);
		  for(int i=1;i<=20;i++) {
			  std::cout << v_spline(i*0.3) <<std::endl;
		  }
		  std::cout << prev_size << std::endl;
		  double target_x = 10;
		  double target_y = s(target_x);
		  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
		  
		  double x_add_on = 0;
		  for(int i=1;i<=num_path_p-previous_path_x.size();i++) {
			  double x_point = 0;
			  double y_point = 0;
			  /*
			  if (previous_path_x.size() == 0) {
				  x_point = x_add_on + (0.1*(ref_t)*(ref_t)*(i+1)*(i+1));
				  y_point = s(x_point);
				  set_vel = 11.6;
			  } else {
				  set_vel += (ref_vel-set_vel)/100;
				  double N = (target_dist/(ref_t*set_vel));
				  x_point = x_add_on+(target_x)/N;
				  y_point = s(x_point);
			  }
			  */
			  double inc_d = v_spline((i+1)*ref_t)*ref_t;
			  x_point = x_add_on + inc_d;
			  y_point = s(x_point);
			  x_add_on = x_point;
			  
			  double x_ref = x_point;
			  double y_ref = y_point;
			  
			  x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
			  y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));
			  
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