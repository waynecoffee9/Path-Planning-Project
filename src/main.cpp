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
Vehicle Ego(1, 0.0, 0.0, 0.0, 0.0,"CS");

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
  // declare some constants
  const double TRACK_L = 6945.554; // track length
  const double MAX_VEL = 21.9; // reference max velocity m/s
  const int SHORT_PATH_PTS = 10; // short path planning points
  const int NORM_PATH_PTS = 100; // normal path planning points
  const int START_PATH_PTS = 300; // startup path planning points
  const double MPH_MPS = 2.236936; // convert from mph to meters per sec
  const double LANE_WIDTH = 4.0; // lane width
  const vector<double> FUSION_D = {90.0,-15}; // vehicle detection range from sensor fusion
  const vector<double> ALERT_D = {18.0, 14.0}; // distances of vehicle in front of Ego to start slowing down
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &TRACK_L, &NORM_PATH_PTS,
			   &SHORT_PATH_PTS, &MAX_VEL, &MPH_MPS, &LANE_WIDTH, &FUSION_D, 
			   &ALERT_D, &START_PATH_PTS]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
  

	// reference delta time between two points
	double ref_t = 0.02;

	// number of path planning points to set at a time.  
	// if environment causes change of action (e.g. change in velocity and lane), reduce the previous
	// path points to 5, and add modified path planning points based on appropriate maneuver
	// for acceleration at the beginning, assign 300 points which represents 6 seconds.
	Ego.num_path_p = NORM_PATH_PTS;
	// clear path planning variables from previous cycle
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
		  Ego.speed = car_speed/MPH_MPS;

		  // Ego state default
		  if (Ego.transition==true && Ego.d > Ego.lane*LANE_WIDTH+1.6 && Ego.d < Ego.lane*LANE_WIDTH+2.4) {
			  Ego.transition = false;
		  } else if (Ego.state == "KL") Ego.state = "CS";
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

		  // declare nearby vehicle predictions
		  map<int ,vector<Vehicle> > car_pred_map;
		  
		  // search for near by vehicles and predict movements within 2 seconds
		  for (int i=0;i<sensor_fusion.size();i++) {
			  vector<double> car = sensor_fusion[i];
			  double bumper_dis = car[5] - car_s;
			  if (bumper_dis < FUSION_D[0] && bumper_dis > FUSION_D[1]) {
				  Vehicle other_car(0,car[3], car[4], car[5], car[6]);
				  other_car.x = car[1];
				  other_car.y = car[2];
				  car_pred_map[car[0]] = other_car.generate_predictions(map_waypoints_s,map_waypoints_x,map_waypoints_y);
			  }
		  }
		  // overwrite state if Ego is getting too close behind a car
		  if (Ego.state != "PLCL" && Ego.state != "PLCR" && !Ego.transition) {
			  for (int i=0;i<sensor_fusion.size();i++) {
				  vector<double> car = sensor_fusion[i];
				  double bumper_dis = car[5] - car_s;
				  if (car[5] > car_s && bumper_dis < ALERT_D[0]) {
					  if (car[6] > (Ego.lane*4-0.5) && car[6] < (Ego.lane*4+4.5)) {
						  double speed = sqrt(car[3]*car[3]+car[4]*car[4]);
						  if (speed < Ego.ref_vel) {
							  double del_speed = Ego.ref_vel - speed;
							  if (bumper_dis < ALERT_D[1]) {
								  Ego.ref_vel = speed * bumper_dis / ALERT_D[1];
							  } else Ego.ref_vel = speed;
							  Ego.num_path_p = std::min(int(bumper_dis / del_speed / ref_t/5+SHORT_PATH_PTS),NORM_PATH_PTS/2+SHORT_PATH_PTS);
							  Ego.state = "KL";
						  }
					  }
				  }
			  }
		  }
		  // reduce path planning points down for lane change or changing speed.
		  if ((Ego.state == "KL" || Ego.state == "PLCL" || Ego.state == "PLCR") && !Ego.transition) {
			  if (previous_path_x.size() > SHORT_PATH_PTS) {
				  std::vector<double> subx(&previous_path_x[0],&previous_path_x[SHORT_PATH_PTS]);
				  std::vector<double> suby(&previous_path_y[0],&previous_path_y[SHORT_PATH_PTS]);
				  previous_path_x = subx;
				  previous_path_y = suby;
			  }
		  }
		  // store some variables in Ego object
		  Ego.prev_size = previous_path_x.size();
		  Ego.ref_x = car_x;
		  Ego.ref_y = car_y;
		  Ego.ref_yaw = deg2rad(car_yaw);
		  // collect base points for spline generation
		  // initial condition where car is stopped
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
			  Ego.num_path_p = START_PATH_PTS;
		  }
		  // states other than initial condition
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
		  // fill path points with previous planned points
		  for (int i = 0; i < previous_path_x.size(); ++i) {
			  Ego.next_x_vals.push_back(previous_path_x[i]);
			  Ego.next_y_vals.push_back(previous_path_y[i]);
		  }
		  // call the function below within Ego to determine the best next state, based on surrounding traffic.
		  // resulting x and y path planning points are stored in Ego.
		  Ego = Ego.choose_next_state(car_pred_map, Ego, ref_t, NORM_PATH_PTS, map_waypoints_s,map_waypoints_x,map_waypoints_y);
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