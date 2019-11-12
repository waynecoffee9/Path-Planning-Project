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
  // reference velocity
  double ref_vel = 0.0;
  const double MAX_VEL = 21.9;
  const int SHORT_PATH_POINTS = 10;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &TRACK_L,
			   &SHORT_PATH_POINTS, &MAX_VEL]
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
		  ref_vel = MAX_VEL;

		  num_path_p = 50;

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
					  if (speed < ref_vel) {
						  double del_dist = car[5] - car_s;
						  double del_speed = ref_vel - speed;
						  if (bumper_dis < 15) {
							  ref_vel = speed * bumper_dis / 15;
						  } else ref_vel = speed;
						  num_path_p = std::min(del_dist / del_speed / ref_t/5+SHORT_PATH_POINTS,50.0+SHORT_PATH_POINTS);
						  stat_change = true;
						  //std::cout << ref_vel << std::endl;
						  //std::cout << num_path_p << std::endl;
					  }
				  }					  
			  }
		  }

		  // reduce path planning points down
		  if (stat_change) {
			  if (previous_path_x.size() > SHORT_PATH_POINTS) {
				  std::vector<double> subx(&previous_path_x[0],&previous_path_x[SHORT_PATH_POINTS]);
				  std::vector<double> suby(&previous_path_y[0],&previous_path_y[SHORT_PATH_POINTS]);
				  previous_path_x = subx;
				  previous_path_y = suby;
				  //previous_path_x.erase(previous_path_x.begin(), previous_path_x.begin()+20);
				  //previous_path_y.erase(previous_path_x.begin(), previous_path_x.begin()+20);
			  }
		  }
		  
		  
		  int prev_size = previous_path_x.size();
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
			  double prev_car_x = car_x - cos(ref_yaw);
			  double prev_car_y = car_y - sin(ref_yaw);
			  
			  ptsx.push_back(prev_car_x);
			  ptsx.push_back(car_x);
			  
			  ptsy.push_back(prev_car_y);
			  ptsy.push_back(car_y);
			  
			  v_spline_y.push_back(0.0);
			  //v_spline_y.push_back(0.0);
			  v_spline_x.push_back(0.0);
			  //v_spline_x.push_back(0.0);
			  
			  stat_change = true;
			  num_path_p = 200;
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
			  //double vel_o = distance(ref_x_prev,ref_y_prev,previous_path_x[prev_size-3],previous_path_y[prev_size-3])/ref_t;
			  v_spline_y.push_back(vel_f);
			  //v_spline_y.push_back(vel_f);
			  //v_spline_x.push_back(-ref_t);
			  v_spline_x.push_back(0.0);
			  
		  }
		  double test = prev_size*ref_t*20;

		  vector<double> next_wp0 = getXY(car_s+test+10,(2+4*Ego.lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		  vector<double> next_wp1 = getXY(car_s+test+40,(2+4*Ego.lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		  vector<double> next_wp2 = getXY(car_s+test+70,(2+4*Ego.lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		  
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
		  double vel_f = v_spline_y.back();
		  std::cout << "ref  " << ref_vel << std::endl;
		  std::cout << "last " << vel_f << std::endl;
		  v_spline_y.push_back((ref_vel-vel_f)*0.2+vel_f);
		  v_spline_y.push_back((ref_vel+vel_f)/2.0);
		  v_spline_y.push_back((ref_vel-vel_f)*0.8+vel_f);
		  v_spline_y.push_back(ref_vel);
		  v_spline_x.push_back(num_path_p*ref_t*0.25);
		  v_spline_x.push_back(num_path_p*ref_t*0.5);
		  v_spline_x.push_back(num_path_p*ref_t*0.75);
		  v_spline_x.push_back(num_path_p*ref_t);
		  v_spline.set_points(v_spline_x, v_spline_y);

		  double target_x = 100;
		  double target_y = s(target_x);
		  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
		  
		  double x_add_on = 0;
		  double s_add_on = 0;
		  
		  if (previous_path_x.size() < 50) {
			  for(int i=1;i<=num_path_p-previous_path_x.size();i++) {
				  double x_point = 0;
				  double y_point = 0;

				  if (stat_change) {
					  double inc_d = (v_spline((i)*ref_t))*ref_t;
					  //std::cout << inc_d <<std::endl;
					  x_point = x_add_on + inc_d;
					  y_point = s(x_point);
				  }
				  else {
					  //double N = (target_dist/(ref_t*ref_vel));
					  //x_point = x_add_on+(target_x)/N;
					  x_point = x_add_on+(ref_vel*ref_t);
					  y_point = s(x_point);
					  //std::cout << (ref_vel*ref_t) <<std::endl;
				  }


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