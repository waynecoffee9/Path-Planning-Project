#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include "helpers.h"

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, double vx, double vy, double s, double d, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  Vehicle choose_next_state(map<int, vector<Vehicle>> &predictions,
									Vehicle &Ego, double &ref_t, 
									const int &NORM_PATH_PTS,
									const vector<double> &maps_s, 
									const vector<double> &maps_x, 
									const vector<double> &maps_y);

  vector<string> successor_states();

  void generate_trajectory(string successor_state, Vehicle &Ego, 
										double &ref_t, const int &NORM_PATH_PTS,
										const vector<double> &maps_s, 
										const vector<double> &maps_x, 
										const vector<double> &maps_y,
                                        map<int, vector<Vehicle>> &predictions);

  vector<double> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  void keep_lane_trajectory(Vehicle &Ego, double &ref_t, const int &NORM_PATH_PTS,
							const vector<double> &maps_s, 
							const vector<double> &maps_x, 
							const vector<double> &maps_y);

  void lane_change_trajectory(Vehicle &Ego, double &ref_t, const int &NORM_PATH_PTS,
										const vector<double> &maps_s, 
										const vector<double> &maps_x, 
										const vector<double> &maps_y);

  vector<Vehicle> prep_lane_change_trajectory(string state, 
                                              map<int, vector<Vehicle>> &predictions);

  void increment(int dt);

  double position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane, 
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane, 
                         Vehicle &rVehicle);

  vector<Vehicle> generate_predictions(const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y, int horizon=2);

  void realize_next_state(vector<Vehicle> &trajectory);

  void configure(vector<int> &road_data);
  
  void clean();

  double get_v();
  
  vector<vector<int>> closest_car_in_lane(Vehicle &Ego, map<int, vector<Vehicle>> &predictions);
  
  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, 
                                     {"LCR", -1}, {"PLCR", -1}};
  
  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int id, lane, goal_lane, goal_s, lanes_available, prev_size, num_path_p;

  double x, y, vx, vy, s, d, target_speed, max_acceleration, vel_f, ref_vel;
  
  double ref_x, ref_y, ref_yaw, speed;
  
  vector<double> next_x_vals, next_y_vals, v_spline_x, v_spline_y, ptsx, ptsy;
  
  bool transition;

  string state;
};

#endif  // VEHICLE_H