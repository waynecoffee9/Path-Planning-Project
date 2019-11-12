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
#include <cmath>
#include "vehicle.h"
#include <map>
#include <iterator>
#include <algorithm>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
// declare vehicle object to predict trajectories, cost functions, states, etc.
Vehicle Ego(0, 0.0, 0.0, 0.0, 0.0,"KL");

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
  // reference velocity
  const double MAX_VEL = 21.9;
  const int SHORT_PATH_PTS = 10;
  const int NORM_PATH_PTS = 50;
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
  Ego.lane = 1;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &TRACK_L, &NORM_PATH_PTS,
			   &SHORT_PATH_PTS, &MAX_VEL]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
  

	// reference delta time between two points
	double ref_t = 0.02;
	// Ego state default
	Ego.state = "CS";
	// number of path planning points to set at a time.  Default is 50.  
	// if environment causes change of action (e.g. change in velocity and lane), reduce the previous
	// path points to 5, and add modified path planning points based on appropriate maneuver
	// for acceleration at the beginning, assign 300 points which represents 6 seconds.
	Ego.num_path_p = NORM_PATH_PTS;
	Ego.clean();

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
		  Ego.s = car_s;
		  Ego.d = car_d;
		  Ego.x = car_x;
		  Ego.y = car_y;

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
		  Ego.ref_vel = MAX_VEL;

		  vector<string> possible_states = Ego.successor_states();
		  
		  // declare nearby vehicle predictions
		  map<int ,vector<Vehicle> > car_pred_map;
		  
		  // search for near by vehicles and predict movements within 2 seconds
		  for (int i=0;i<sensor_fusion.size();i++) {
			  vector<double> car = sensor_fusion[i];
			  double bumper_dis = car[5] - car_s;
			  if (bumper_dis < 50 && bumper_dis > -20) {
				  Vehicle other_car(0,car[3], car[4], car[5], car[6]);
				  other_car.x = car[1];
				  other_car.y = car[2];
				  car_pred_map[car[0]] = other_car.generate_predictions(map_waypoints_s,map_waypoints_x,map_waypoints_y);
			  }
		  }
		  
		  // list all Ego's possible movements
		  //string next_state = '';
		  float cost = 9999999999;
		  for (int i = 0; i < possible_states.size(); i++) {

		  }
		  // keep lane
		  double cost_KL = 0;
		  for (map<int, vector<Vehicle>>::iterator it = car_pred_map.begin(); it != car_pred_map.end(); ++it) {
			  //if (it->second[2] < MAX_VEL) {}
			  
		  }
		  
		  
		  
		  for (int i=0;i<sensor_fusion.size();i++) {
			  vector<double> car = sensor_fusion[i];
			  double bumper_dis = car[5] - car_s;
			  if (car[5] > car_s && bumper_dis < 20) {
				  if (car[6] > (car_d-2.5) && car[6] < (car_d+2.5)) {
					  double speed = sqrt(car[3]*car[3]+car[4]*car[4]);
					  if (speed < Ego.ref_vel) {
						  double del_dist = car[5] - car_s;
						  double del_speed = Ego.ref_vel - speed;
						  if (bumper_dis < 15) {
							  Ego.ref_vel = speed * bumper_dis / 15;
						  } else Ego.ref_vel = speed;
						  Ego.num_path_p = std::min(del_dist / del_speed / ref_t/7+SHORT_PATH_PTS,50.0+SHORT_PATH_PTS);
						  Ego.state = "KL";
					  }
				  }					  
			  }
		  }

		  // reduce path planning points down
		  if (Ego.state == "KL") {
			  if (previous_path_x.size() > SHORT_PATH_PTS) {
				  std::vector<double> subx(&previous_path_x[0],&previous_path_x[SHORT_PATH_PTS]);
				  std::vector<double> suby(&previous_path_y[0],&previous_path_y[SHORT_PATH_PTS]);
				  previous_path_x = subx;
				  previous_path_y = suby;
				  //previous_path_x.erase(previous_path_x.begin(), previous_path_x.begin()+20);
				  //previous_path_y.erase(previous_path_x.begin(), previous_path_x.begin()+20);
			  }
		  }
		  
		  Ego.prev_size = previous_path_x.size();
		  
		  Ego.ref_x = car_x;
		  Ego.ref_y = car_y;
		  Ego.ref_yaw = deg2rad(car_yaw);
		  
		  if(Ego.prev_size < 2) {
			  double prev_car_x = car_x - cos(Ego.ref_yaw);
			  double prev_car_y = car_y - sin(Ego.ref_yaw);
			  
			  Ego.ptsx.push_back(prev_car_x);
			  Ego.ptsx.push_back(car_x);
			  
			  Ego.ptsy.push_back(prev_car_y);
			  Ego.ptsy.push_back(car_y);
			  
			  Ego.v_spline_y.push_back(0.0);
			  Ego.v_spline_x.push_back(0.0);
			  
			  Ego.state = "KL";
			  Ego.vel_f = 0;
			  Ego.num_path_p = 300;
		  }
		  else {
			  Ego.ref_x = previous_path_x[Ego.prev_size-1];
			  Ego.ref_y = previous_path_y[Ego.prev_size-1];
			  
			  double ref_x_prev = previous_path_x[Ego.prev_size-2];
			  double ref_y_prev = previous_path_y[Ego.prev_size-2];
			  Ego.ref_yaw = atan2(Ego.ref_y-ref_y_prev,Ego.ref_x-ref_x_prev);
			  
			  Ego.ptsx.push_back(ref_x_prev);
			  Ego.ptsx.push_back(Ego.ref_x);
			  
			  Ego.ptsy.push_back(ref_y_prev);
			  Ego.ptsy.push_back(Ego.ref_y);
			  
			  Ego.vel_f = distance(Ego.ref_x,Ego.ref_y,ref_x_prev,ref_y_prev)/ref_t;
			  Ego.v_spline_y.push_back(Ego.vel_f);
			  Ego.v_spline_x.push_back(0.0);
			  
		  }
		  /*
		  double len = Ego.prev_size*ref_t*20;

		  vector<double> next_wp0 = getXY(Ego.s + len + 10,(2+4*Ego.lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		  vector<double> next_wp1 = getXY(Ego.s + len + 40,(2+4*Ego.lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		  vector<double> next_wp2 = getXY(Ego.s + len + 70,(2+4*Ego.lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		  
		  Ego.ptsx.push_back(next_wp0[0]);
		  Ego.ptsx.push_back(next_wp1[0]);
		  Ego.ptsx.push_back(next_wp2[0]);
		  
		  Ego.ptsy.push_back(next_wp0[1]);
		  Ego.ptsy.push_back(next_wp1[1]);
		  Ego.ptsy.push_back(next_wp2[1]);
		  
		  // coordinate transformation
		  double shift_x, shift_y;
		  
		  for (int i = 0; i < Ego.ptsx.size(); i++) {
			  shift_x = Ego.ptsx[i]-Ego.ref_x;
			  shift_y = Ego.ptsy[i]-Ego.ref_y;
			  
			  Ego.ptsx[i] = (shift_x * cos(-Ego.ref_yaw)-shift_y * sin(-Ego.ref_yaw));
			  Ego.ptsy[i] = (shift_x * sin(-Ego.ref_yaw)+shift_y * cos(-Ego.ref_yaw));
			  
		  }
		  
		  tk::spline s;
		  // fill next x and y points with previous path points first
		  s.set_points(Ego.ptsx, Ego.ptsy);
		  */
		  for (int i = 0; i < previous_path_x.size(); ++i) {
			  Ego.next_x_vals.push_back(previous_path_x[i]);
			  Ego.next_y_vals.push_back(previous_path_y[i]);
		  }
		  /*
		  // generate spline for velocity increase/decrease over time
		  tk::spline v_spline;
		  Ego.vel_f = Ego.v_spline_y.back();
		  Ego.v_spline_y.push_back((Ego.ref_vel-Ego.vel_f)*0.2+Ego.vel_f);
		  Ego.v_spline_y.push_back((Ego.ref_vel+Ego.vel_f)/2.0);
		  Ego.v_spline_y.push_back((Ego.ref_vel-Ego.vel_f)*0.8+Ego.vel_f);
		  Ego.v_spline_y.push_back(Ego.ref_vel);
		  Ego.v_spline_x.push_back(Ego.num_path_p*ref_t*0.25);
		  Ego.v_spline_x.push_back(Ego.num_path_p*ref_t*0.5);
		  Ego.v_spline_x.push_back(Ego.num_path_p*ref_t*0.75);
		  Ego.v_spline_x.push_back(Ego.num_path_p*ref_t);
		  v_spline.set_points(Ego.v_spline_x, Ego.v_spline_y, false);
		  //std::cout << (Ego.ref_vel-Ego.vel_f)*0.2+Ego.vel_f << std::endl;
		  //std::cout << (Ego.ref_vel+Ego.vel_f)/2.0 << std::endl;
		  //std::cout << (Ego.ref_vel-Ego.vel_f)*0.8+Ego.vel_f << std::endl;
		  //std::cout << Ego.ref_vel << std::endl;
		  double target_x = 100;
		  double target_y = s(target_x);
		  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
		  
		  double x_add_on = 0;
		  
		  if (previous_path_x.size() < 50) {
			  for(int i=1;i<=Ego.num_path_p-previous_path_x.size();i++) {
				  double x_point = 0;
				  double y_point = 0;

				  if (Ego.state == "KL") {
					  double inc_d = (v_spline((i)*ref_t))*ref_t;

					  x_point = x_add_on + inc_d;
					  y_point = s(x_point);
					  if (i>1) {
						  shift_x = Ego.next_x_vals.back()-Ego.ref_x;
						  shift_y = Ego.next_y_vals.back()-Ego.ref_y;
						  
						  double x0 = (shift_x * cos(-Ego.ref_yaw)-shift_y * sin(-Ego.ref_yaw));
						  double y0 = (shift_x * sin(-Ego.ref_yaw)+shift_y * cos(-Ego.ref_yaw));

						  double angle = atan2(y_point - y0, x_point - x0);
						  
						  x_point = x_add_on + cos(angle) * inc_d;
						  y_point = s(x_point);
						  
					  }
				  }
				  else {
					  //double N = (target_dist/(ref_t*ref_vel));
					  //x_point = x_add_on+(target_x)/N;
					  double add_vel = std::min(Ego.vel_f+0.1, Ego.ref_vel);
					  x_point = x_add_on+(add_vel*ref_t);
					  y_point = s(x_point);
					  //std::cout << (ref_vel*ref_t) <<std::endl;
				  }

				  x_add_on = x_point;
				  
				  double x_ref = x_point;
				  double y_ref = y_point;
				  
				  x_point = (x_ref * cos(Ego.ref_yaw)-y_ref*sin(Ego.ref_yaw));
				  y_point = (x_ref * sin(Ego.ref_yaw)+y_ref*cos(Ego.ref_yaw));
				  
				  x_point += Ego.ref_x;
				  y_point += Ego.ref_y;
				  
				  Ego.next_x_vals.push_back(x_point);
				  Ego.next_y_vals.push_back(y_point);
				  
			  }
		  }
		  */
		  Ego.keep_lane_trajectory(Ego, ref_t, NORM_PATH_PTS, map_waypoints_s,map_waypoints_x,map_waypoints_y);
          json msgJson;
          msgJson["next_x"] = Ego.next_x_vals;
          msgJson["next_y"] = Ego.next_y_vals;
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