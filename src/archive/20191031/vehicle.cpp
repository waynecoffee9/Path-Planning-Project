#include "vehicle.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include <math.h>
#include "helpers.h"
#include "spline.h"

using std::string;
using std::vector;

constexpr double pi() { return M_PI; }
const int AVAILABLE_LANES = 3;
// Initializes Vehicle
Vehicle::Vehicle(){
  this->lane = 0;
  this->x = 0;
  this->y = 0;
  this->vx = 0;
  this->vy = 0;
  this->s = 0;
  this->d = 0;
  this->state = "CS";
  this->vel_f = 0;
  this->ref_vel = 0;
  this->ref_x = 0;
  this->ref_y = 0;
  this->ref_yaw = 0;
  this->prev_size = 0;
  this->num_path_p = 0;
  this->v_spline_x;
  this->v_spline_y;
  this->ptsx;
  this->ptsy;
  this->next_x_vals;
  this->next_y_vals;
  this->speed = 0;
  this->transition = false;
  max_acceleration = -1;
}

Vehicle::Vehicle(int lane, double vx, double vy, double s, double d, string state) {
  this->lane = lane;
  this->x = 0;
  this->y = 0;
  this->vx = vx;
  this->vy = vy;
  this->s = s;
  this->d = d;
  this->state = state;
  this->vel_f = 0;
  this->ref_vel = 0;
  this->ref_x = 0;
  this->ref_y = 0;
  this->ref_yaw = 0;
  this->prev_size = 0;
  this->num_path_p = 0;
  this->v_spline_x;
  this->v_spline_y;
  this->ptsx;
  this->ptsy;
  this->next_x_vals;
  this->next_y_vals;
  this->speed = 0;
  this->transition = false;
  max_acceleration = -1;
}

Vehicle::~Vehicle() {}

Vehicle Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions,
											Vehicle &Ego, double &ref_t, 
											const int &NORM_PATH_PTS,
											const vector<double> &maps_s, 
											const vector<double> &maps_x, 
											const vector<double> &maps_y) {
  /**
   * Here you can implement the transition_function code from the Behavior 
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   *
   * TODO: Your solution here.
   */
  vector<string> states = successor_states();
  double cost;
  vector<double> costs;
  vector<vector<Vehicle>> final_trajectories;
  vector<Vehicle> trajectories;

  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
	Vehicle Ego2 = Ego;
    generate_trajectory(*it, Ego2, ref_t, NORM_PATH_PTS,
										maps_s, maps_x, maps_y, predictions);
	trajectories.push_back(Ego2);
    cost = calculate_cost(Ego, Ego2, predictions, *it);
    costs.push_back(cost);
  }

  vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  trajectories[best_idx].state = states[best_idx];

  if (states[best_idx] == "LCL") {
	  trajectories[best_idx].lane -= 1;
	  trajectories[best_idx].transition = true;
  } else if (states[best_idx] == "LCR") {
	  trajectories[best_idx].lane += 1;
	  trajectories[best_idx].transition = true;
  }
  return trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  states.push_back("KL");
  if (!transition) {
	  if(state.compare("KL") == 0 || state.compare("CS") == 0) {
		if (lane > 0) states.push_back("PLCL");
		if (lane < AVAILABLE_LANES - 1) states.push_back("PLCR");
	  } else if (state.compare("PLCL") == 0) {
		states.push_back("PLCL");
		states.push_back("LCL");
	  } else if (state.compare("PLCR") == 0) {
		states.push_back("PLCR");
		states.push_back("LCR");
	  }
  }
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

void Vehicle::generate_trajectory(string successor_state, Vehicle &Ego, 
											double &ref_t, const int &NORM_PATH_PTS,
											const vector<double> &maps_s, 
											const vector<double> &maps_x, 
											const vector<double> &maps_y,
                                            map<int, vector<Vehicle>> &predictions) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  /*
  if (state.compare("KL") == 0 || state.compare("CS") == 0) {
    keep_lane_trajectory(Ego, ref_t, NORM_PATH_PTS, maps_s, maps_x, maps_y);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    Ego.lane -= 1;
	keep_lane_trajectory(Ego, ref_t, NORM_PATH_PTS, maps_s, maps_x, maps_y);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    Ego.lane -= 1;
	keep_lane_trajectory(Ego, ref_t, NORM_PATH_PTS, maps_s, maps_x, maps_y);
  }
  */
  if (successor_state.compare("KL") == 0 || successor_state.compare("CS") == 0) {
    keep_lane_trajectory(Ego, ref_t, NORM_PATH_PTS, maps_s, maps_x, maps_y);
  } else if ((successor_state.compare("PLCL") == 0 || successor_state.compare("PLCR") == 0) && Ego.vel_f > 0) {
	keep_lane_trajectory(Ego, ref_t, NORM_PATH_PTS, maps_s, maps_x, maps_y);
  } else if ((successor_state.compare("LCL") == 0 || successor_state.compare("LCR") == 0) && Ego.vel_f > 0) {
	lane_change_trajectory(Ego, ref_t, NORM_PATH_PTS*2, maps_s, maps_x, maps_y);
  }

}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions, 
                                      int lane) {
  // Gets next timestep kinematics (position, velocity, acceleration) 
  //   for a given lane. Tries to choose the maximum velocity and acceleration, 
  //   given other vehicle positions and accel/velocity constraints.
  double max_velocity_accel_limit = this->max_acceleration + this->vx;
  double new_position;
  double new_velocity;
  double new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      new_velocity = vehicle_ahead.vx;
    } else {
      double max_velocity_in_front = (vehicle_ahead.s - this->s 
                                  - this->preferred_buffer) + vehicle_ahead.vx
                                  - 0.5 * (this->d);
      new_velocity = std::min(std::min(max_velocity_in_front, 
                                       max_velocity_accel_limit), 
                                       this->target_speed);
    }
  } else {
    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
  }
    
  new_accel = new_velocity - this->vx; // Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity + new_accel / 2.0;
    
  return{new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  double next_pos = position_at(1);
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->vx, this->vy, this->s, this->d, this->state), 
                                Vehicle(lane, this->vx, this->vy, this->s, this->d, state)};
  return trajectory;
}

void Vehicle::keep_lane_trajectory(Vehicle &Ego, double &ref_t, const int &NORM_PATH_PTS,
									const vector<double> &maps_s, 
									const vector<double> &maps_x, 
									const vector<double> &maps_y) {
  // Generate a keep lane trajectory.
  // velocity vs time spline
  tk::spline v_spline;

  // anchor points for the velocity spline 
  Ego.v_spline_y.push_back((Ego.ref_vel-Ego.vel_f)*0.2+vel_f);
  Ego.v_spline_y.push_back((Ego.ref_vel+Ego.vel_f)/2.0);
  Ego.v_spline_y.push_back((Ego.ref_vel-Ego.vel_f)*0.8+vel_f);
  Ego.v_spline_y.push_back(Ego.ref_vel);
  Ego.v_spline_x.push_back(Ego.num_path_p*ref_t*0.25);
  Ego.v_spline_x.push_back(Ego.num_path_p*ref_t*0.5);
  Ego.v_spline_x.push_back(Ego.num_path_p*ref_t*0.75);
  Ego.v_spline_x.push_back(Ego.num_path_p*ref_t);

  // set velocity spline
  v_spline.set_points(Ego.v_spline_x, Ego.v_spline_y);
  // anchor points for the path spline
  double len = Ego.prev_size*ref_t*25;
  vector<double> next_wp0 = getXY(Ego.s + len + 10,(2+4*Ego.lane),maps_s,maps_x,maps_y);
  vector<double> next_wp1 = getXY(Ego.s + len + 30,(2+4*Ego.lane),maps_s,maps_x,maps_y);
  vector<double> next_wp2 = getXY(Ego.s + len + 60,(2+4*Ego.lane),maps_s,maps_x,maps_y);

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
  
  // path (x vs y) spline
  tk::spline s;
  // fill next x and y points with previous path points first
  s.set_points(Ego.ptsx, Ego.ptsy);
  
  // generate paht points in xy coordinates
  double x_add_on = 0;
  
  if (Ego.prev_size < NORM_PATH_PTS) {
	  for(int i=1;i<=Ego.num_path_p-Ego.prev_size;i++) {
		  double x_point = 0;
		  double y_point = 0;

		  if (Ego.state != "CS") {
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
			  double add_vel = std::min(Ego.vel_f+0.07, Ego.ref_vel);
			  x_point = x_add_on+(add_vel*ref_t);
			  y_point = s(x_point);
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
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, 
                                                     map<int, vector<Vehicle>> &predictions) {
  // Generate a trajectory preparing for a lane change.
  double new_s;
  double new_v;
  double new_a;
  Vehicle vehicle_behind;
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->vx, this->vy, this->s, this->d, this->state)};
  vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

  if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];    
  } else {
    vector<double> best_kinematics;
    vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
    // Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  trajectory.push_back(Vehicle(this->lane, this->vx, this->vy, this->s, this->d, this->state));
  
  return trajectory;
}

void Vehicle::lane_change_trajectory(Vehicle &Ego, double &ref_t, const int &NORM_PATH_PTS,
												const vector<double> &maps_s, 
												const vector<double> &maps_x, 
												const vector<double> &maps_y) {
  // Generate a lane change trajectory.
  double len = Ego.prev_size*ref_t*25;
  double lane_change_t = NORM_PATH_PTS * ref_t;
  int target_lane;
  if (Ego.state == "PLCL" || Ego.state == "LCL") target_lane = Ego.lane - 1;
  else target_lane = Ego.lane + 1;
  vector<double> next_wp0 = getXY(Ego.s + len + Ego.vel_f*lane_change_t*0.4,(2+4*(target_lane+Ego.lane)/2),maps_s,maps_x,maps_y);
  vector<double> next_wp1 = getXY(Ego.s + len + Ego.vel_f*lane_change_t*0.8,(2+4*target_lane),maps_s,maps_x,maps_y);
  vector<double> next_wp2 = getXY(Ego.s + len + Ego.vel_f*lane_change_t*1.0,(2+4*target_lane),maps_s,maps_x,maps_y);

  Ego.ptsx.push_back(next_wp0[0]);
  Ego.ptsx.push_back(next_wp1[0]);
  Ego.ptsx.push_back(next_wp2[0]);
  
  Ego.ptsy.push_back(next_wp0[1]);
  Ego.ptsy.push_back(next_wp1[1]);
  Ego.ptsy.push_back(next_wp2[1]);

  double shift_x, shift_y;
  for (int i = 0; i < Ego.ptsx.size(); i++) {
	  shift_x = Ego.ptsx[i]-Ego.ref_x;
	  shift_y = Ego.ptsy[i]-Ego.ref_y;
	  
	  Ego.ptsx[i] = (shift_x * cos(-Ego.ref_yaw)-shift_y * sin(-Ego.ref_yaw));
	  Ego.ptsy[i] = (shift_x * sin(-Ego.ref_yaw)+shift_y * cos(-Ego.ref_yaw));
  }

  if (Ego.state == "LCL") Ego.lane -= 1;
  else if (Ego.state == "LCR") Ego.lane += 1;
  // path (x vs y) spline
  tk::spline s;
  // fill next x and y points with previous path points first
  s.set_points(Ego.ptsx, Ego.ptsy);
  
  // generate paht points in xy coordinates
  double x_add_on = 0;
  // 
  if (Ego.prev_size < NORM_PATH_PTS) {
	  for(int i=1;i<=NORM_PATH_PTS-Ego.prev_size;i++) {
		  double x_point = 0;
		  double y_point = 0;

		  //double N = (target_dist/(ref_t*ref_vel));
		  //x_point = x_add_on+(target_x)/N;
		  //double add_vel = std::min(Ego.vel_f+0.07, Ego.ref_vel);
		  x_point = x_add_on+(Ego.vel_f*ref_t);
		  y_point = s(x_point);

		  if (i>1) {
			  shift_x = Ego.next_x_vals.back()-Ego.ref_x;
			  shift_y = Ego.next_y_vals.back()-Ego.ref_y;
			  
			  double x0 = (shift_x * cos(-Ego.ref_yaw)-shift_y * sin(-Ego.ref_yaw));
			  double y0 = (shift_x * sin(-Ego.ref_yaw)+shift_y * cos(-Ego.ref_yaw));

			  double angle = atan2(y_point - y0, x_point - x0);
			  
			  x_point = x_add_on + cos(angle) * (Ego.vel_f*ref_t);
			  y_point = s(x_point);
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
  /*
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies 
  //   that spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
      // If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(this->lane, this->vx, this->vy, this->s, this->d, this->state));
  vector<double> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(Vehicle(this->lane, this->vx, this->vy, this->s, this->d, this->state));
  return trajectory;
  */
  
}

void Vehicle::increment(int dt = 1) {
  this->s = position_at(dt);
}

double Vehicle::position_at(int t) {
  return this->s + this->vx*t + this->d*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions, 
                                 int lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found behind the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s 
        && temp_vehicle.s > max_s) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, 
                                int lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int min_s = this->goal_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s 
        && temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y, int horizon) {
  // Generates predictions for non-ego vehicles to be used in trajectory 
  //   generation for the ego vehicle.
  vector<Vehicle> predictions;
  int original_lane = round((this->d - 2.0) / 4.0);
  string new_state;
  for(int i = 0; i <= horizon; ++i) {
	  double vel = this->get_v();
	  // check if direction of resultant velocity is quite different from road direction
	  // if they are, it is likely the car is in process of changing lane
	  double dir_v = atan2(vy, vx);
	  vector<double> xy = getXY(this->s + 0.5, this->d, maps_s, maps_x, maps_y);
	  double dir_road = atan2(xy[1] - this->y, xy[0] - this->x);
	  double new_d = this->d - sin(dir_v - dir_road) * vel * i;
	  int lane = round((new_d - 2.0) / 4.0);
	  if (lane < 3 && lane > -1) {
		  if (lane > original_lane) new_state = "LCR";
		  else if (lane < original_lane) new_state = "LCL";
		  else new_state = "KL";
	  }

    predictions.push_back(Vehicle(original_lane, this->vx, this->vy, this->s + vel*(i), new_d, new_state));
  }
  
  return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->vx = next_state.vx;
  this->d = next_state.d;
}

void Vehicle::configure(vector<int> &road_data) {
  // Called by simulator before simulation begins. Sets various parameters which
  //   will impact the ego vehicle.
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}

double Vehicle::get_v() {
  return sqrt(this->vx * this->vx + this->vy * this->vy);
}

vector<vector<int>> Vehicle::closest_car_in_lane(Vehicle &Ego, map<int, vector<Vehicle>> &predictions) {
	vector<vector<int>> final_id;
	for (int i=0; i<AVAILABLE_LANES; i++) {
		vector<double> vehicle_dist;
		vector<int> vehicle_id;
		double target_s = Ego.s + Ego.speed * 2.0;
		int minElementIndex;
		for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
			// check for other vehicle lane change
			
			int lane_change = 0;
			/*
			if (it->second[1].state == "LCL" && it->second[2].state == "LCL") {
				lane_change--;
			} else if (it->second[1].state == "LCR" && it->second[2].state == "LCR") {
				lane_change++;
			}
			*/
			// check if Ego and other vehicle are going to be at the same lane in 2 sec.
			// check for closest vehicle in front of Ego

			if (it->second[0].lane + lane_change == i && it->second[0].s > Ego.s - 8.0) {
				vehicle_dist.push_back(it->second[2].s - target_s);
				vehicle_id.push_back(it->first);
			}
		}

		if (vehicle_dist.size() != 0) {
			minElementIndex = min_element(vehicle_dist.begin(), vehicle_dist.end()) - vehicle_dist.begin();
			final_id.push_back({vehicle_id[minElementIndex],int(vehicle_dist[minElementIndex])});
		} else final_id.push_back({999,60});
	}
	return final_id;

}

void Vehicle::clean() {
	this->v_spline_x.clear();
	this->v_spline_y.clear();
	this->ptsx.clear();
	this->ptsy.clear();
	this->next_x_vals.clear();
	this->next_y_vals.clear();
	
}