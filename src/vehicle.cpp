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
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   */
  double cost;
  vector<double> costs;
  // find all possible next states based on current state 
  vector<string> states = successor_states();
  vector<Vehicle> trajectories;
  // iterate through possible states, calculate each path and cost
  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
	Vehicle Ego2 = Ego;
    generate_trajectory(*it, Ego2, ref_t, NORM_PATH_PTS,
										maps_s, maps_x, maps_y, predictions);
	trajectories.push_back(Ego2);
    cost = calculate_cost(Ego, Ego2, predictions, *it);
    costs.push_back(cost);
  }
  // find the lowest cost and execute it
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
  if (successor_state.compare("KL") == 0 || successor_state.compare("CS") == 0) {
    keep_lane_trajectory(Ego, ref_t, NORM_PATH_PTS, maps_s, maps_x, maps_y);
  } else if ((successor_state.compare("PLCL") == 0 || successor_state.compare("PLCR") == 0) && Ego.vel_f > 0) {
	keep_lane_trajectory(Ego, ref_t, NORM_PATH_PTS, maps_s, maps_x, maps_y);
  } else if ((successor_state.compare("LCL") == 0 || successor_state.compare("LCR") == 0) && Ego.vel_f > 0) {
	lane_change_trajectory(Ego, ref_t, NORM_PATH_PTS*2, maps_s, maps_x, maps_y);
  }

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
  vector<double> next_wp0 = getXY(Ego.s + len + 20,(2+4*Ego.lane),maps_s,maps_x,maps_y);
  vector<double> next_wp1 = getXY(Ego.s + len + 40,(2+4*Ego.lane),maps_s,maps_x,maps_y);
  vector<double> next_wp2 = getXY(Ego.s + len + 60,(2+4*Ego.lane),maps_s,maps_x,maps_y);
  vector<double> next_wp3 = getXY(Ego.s + len + 90,(2+4*Ego.lane),maps_s,maps_x,maps_y);
  vector<double> next_wp4 = getXY(Ego.s + len + 120,(2+4*Ego.lane),maps_s,maps_x,maps_y);
  
  Ego.ptsx.push_back(next_wp0[0]);
  Ego.ptsx.push_back(next_wp1[0]);
  Ego.ptsx.push_back(next_wp2[0]);
  Ego.ptsx.push_back(next_wp3[0]);
  Ego.ptsx.push_back(next_wp4[0]);
  
  Ego.ptsy.push_back(next_wp0[1]);
  Ego.ptsy.push_back(next_wp1[1]);
  Ego.ptsy.push_back(next_wp2[1]);
  Ego.ptsy.push_back(next_wp3[1]);
  Ego.ptsy.push_back(next_wp4[1]);
  
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
  // add path points
  if (Ego.prev_size < NORM_PATH_PTS) {
	  for(int i=1;i<=Ego.num_path_p-Ego.prev_size;i++) {
		  double x_point = 0;
		  double y_point = 0;
		  // for speed change path points
		  if (Ego.state != "CS") {
			  double inc_d = (v_spline((i)*ref_t))*ref_t;

			  x_point = x_add_on + inc_d;
			  y_point = s(x_point);
			  // when car is going around a curve, find the x, y components to keep speed in control
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
			  // if speed is lower than reference, speed up
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
  // anchor points for the path spline
  vector<double> next_wp0 = getXY(Ego.s + len + Ego.vel_f*lane_change_t*0.3,(2+4*(target_lane+Ego.lane)/2),maps_s,maps_x,maps_y);
  vector<double> next_wp1 = getXY(Ego.s + len + Ego.vel_f*lane_change_t*0.6,(2+4*target_lane),maps_s,maps_x,maps_y);
  vector<double> next_wp2 = getXY(Ego.s + len + Ego.vel_f*lane_change_t*0.8,(2+4*target_lane),maps_s,maps_x,maps_y);
  vector<double> next_wp3 = getXY(Ego.s + len + Ego.vel_f*lane_change_t*1.0,(2+4*target_lane),maps_s,maps_x,maps_y);

  Ego.ptsx.push_back(next_wp0[0]);
  Ego.ptsx.push_back(next_wp1[0]);
  Ego.ptsx.push_back(next_wp2[0]);
  Ego.ptsx.push_back(next_wp3[0]);
  
  Ego.ptsy.push_back(next_wp0[1]);
  Ego.ptsy.push_back(next_wp1[1]);
  Ego.ptsy.push_back(next_wp2[1]);
  Ego.ptsy.push_back(next_wp3[1]);
  // coordinate transformation
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
  // add path points
  if (Ego.prev_size < NORM_PATH_PTS) {
	  for(int i=1;i<=NORM_PATH_PTS-Ego.prev_size;i++) {
		  double x_point = 0;
		  double y_point = 0;
		  x_point = x_add_on+(Ego.vel_f*ref_t);
		  y_point = s(x_point);
		  // when car is going around a curve, find the x, y components to keep speed in control
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
	// update s, d, and state
    predictions.push_back(Vehicle(original_lane, this->vx, this->vy, this->s + vel*(i), new_d, new_state));
  }
  
  return predictions;
}
// calculate vehicle speed
double Vehicle::get_v() {
  return sqrt(this->vx * this->vx + this->vy * this->vy);
}

// go through list of nearby vehicles, and find the closest vehicle in front of Ego in each lane
vector<vector<int>> Vehicle::closest_car_in_lane(Vehicle &Ego, map<int, vector<Vehicle>> &predictions) {
	vector<vector<int>> final_id;
	// iterate through all lanes
	for (int i=0; i<AVAILABLE_LANES; i++) {
		vector<double> vehicle_dist;
		vector<int> vehicle_id;
		double target_s = Ego.s + Ego.speed * 2.0;
		int minElementIndex;
		// iterate through nearby vehicles
		for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
			if (it->second[0].lane == i && it->second[0].s > Ego.s - 8.0) {
				vehicle_dist.push_back(it->second[2].s - target_s);
				vehicle_id.push_back(it->first);
			}
		}
		// if any vehicle found, add it to the list along with its id and distance from Ego
		// if no vehicle found, use 999 as id, and 60 as distance
		if (vehicle_dist.size() != 0) {
			minElementIndex = min_element(vehicle_dist.begin(), vehicle_dist.end()) - vehicle_dist.begin();
			final_id.push_back({vehicle_id[minElementIndex],int(vehicle_dist[minElementIndex])});
		} else final_id.push_back({999,100});
	}
	return final_id;

}

// clear variables for path point generation
void Vehicle::clean() {
	this->v_spline_x.clear();
	this->v_spline_y.clear();
	this->ptsx.clear();
	this->ptsy.clear();
	this->next_x_vals.clear();
	this->next_y_vals.clear();
}