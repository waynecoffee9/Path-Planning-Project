#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"
#include <iostream>

using std::string;
using std::vector;

/**
 * TODO: change weights for cost functions.
 */
const float SAFETY = pow(10, 9);
const float EFFICIENCY = pow(10, 5);

// Here we have provided two possible suggestions for cost functions, but feel 
//   free to use your own! The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be very useful in 
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.

float goal_distance_cost(const Vehicle &vehicle, 
                         const vector<Vehicle> &trajectory, 
                         const map<int, vector<Vehicle>> &predictions, 
                         map<string, float> &data) {
  // Cost increases based on distance of intended lane (for planning a lane 
  //   change) and final lane of trajectory.
  // Cost of being out of goal lane also becomes larger as vehicle approaches 
  //   goal distance.
  // This function is very similar to what you have already implemented in the 
  //   "Implement a Cost Function in C++" quiz.
  float cost;
  float distance = data["distance_to_goal"];
  if (distance > 0) {
    cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] 
         - data["final_lane"]) / distance));
  } else {
    cost = 1;
  }

  return cost;
}

float inefficiency_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, vector<Vehicle>> &predictions, 
                        map<string, float> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane. 
  // This function is very similar to what you have already implemented in 
  //   the "Implement a Second Cost Function in C++" quiz.
  float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = vehicle.target_speed;
  }

  float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
    proposed_speed_final = vehicle.target_speed;
  }
    
  float cost = (2.0*vehicle.target_speed - proposed_speed_intended 
             - proposed_speed_final)/vehicle.target_speed;

  return cost;
}

float lane_speed(const map<int, vector<Vehicle>> &predictions, int lane) {
  // All non ego vehicles in a lane have the same speed, so to get the speed 
  //   limit for a lane, we can just find one vehicle in that lane.
  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    int key = it->first;
    Vehicle vehicle = it->second[0];
    if (vehicle.lane == lane && key != -1) {
      return vehicle.vx;
    }
  }
  // Found no vehicle in the lane
  return -1.0;
}

float calculate_cost(Vehicle &Ego, const Vehicle &trajectory,
                     map<int, vector<Vehicle>> &predictions, 
                     const string state) {
  // Sum weighted cost functions to get total cost for trajectory.
  // Safety and efficiency are considered.
  // KL: efficiency
  // PLCL/PLCR: safety and efficiency
  float cost = 0.0;
  double target_s = Ego.s + Ego.speed * 2.0;

  if (state == "KL") {
	  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		  // check for lane change
		  int lane_change = 0;
		  if (it->second[1].state == "LCL" && it->second[2].state == "LCL") {
			  lane_change--;
		  } else if (it->second[1].state == "LCR" && it->second[2].state == "LCR") {
			  lane_change++;
		  }
		  // check for vehicles in front of Ego
		  if (it->second[0].s > Ego.s) {
			  // check for vehicles in same lane as Ego
			  if ((it->second[0].lane + lane_change) == Ego.lane && EFFICIENCY/it->second[0].get_v() > cost) {
				  cost = EFFICIENCY/(it->second[0].get_v());
			  }
		  }
	  }
	  //std::cout << "KL cost:" << cost << std::endl;
  } else if (state == "PLCL" || state == "PLCR" || state == "LCL" || state == "LCR") {
	  int target_lane;
	  if (state == "PLCL" || state == "LCL") target_lane = Ego.lane - 1;
	  else target_lane = Ego.lane + 1;
	  // find closest vehicles in front of Ego in all lanes
	  vector<vector<int>> closest_vehicle_list = Ego.closest_car_in_lane(Ego, predictions);
	  // find the leading vehicle from the list above
	  int leading_lane=0;
	  int leading_dist=-100;
	  for (int i=0; i<closest_vehicle_list.size(); i++) {
		  if (closest_vehicle_list[i][1] > leading_dist) {
			  leading_lane = i;
			  leading_dist = closest_vehicle_list[i][1];
		  }
	  }
	  // distance between leading vehicle and vehicle in front of Ego.
	  int del_dist = closest_vehicle_list[leading_lane][1] - closest_vehicle_list[Ego.lane][1];
	  if (del_dist > 15 && (target_lane == leading_lane || abs(Ego.lane - leading_lane) == 2)) {
		  cost = float(EFFICIENCY/del_dist);
	  } else cost = EFFICIENCY;
	  //std::cout << state << " dist:" << del_dist << std::endl;
	  //std::cout << state << " lead lane:" << leading_lane << std::endl;
	  //std::cout << state << " target lane dist:" << closest_vehicle_list[Ego.lane][1] << std::endl;
	  //std::cout << state << " cost:" << cost << std::endl;
	  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		  // check for safety: are we going to hit another vehicle?
		  // check for other vehicle lane change
		  int lane_change = 0;
		  if (it->second[1].state == "LCL" && it->second[2].state == "LCL") {
			  lane_change--;
		  } else if (it->second[1].state == "LCR" && it->second[2].state == "LCR") {
			  lane_change++;
		  }

		  // check for other vehicles in the same target lane as Ego.
		  if ((it->second[0].lane + lane_change) == target_lane) {
			  if (it->second[0].s > (Ego.s - 10.0) && it->second[0].s < (Ego.s + 14.0)) {
				  cost = SAFETY;
			  }
			  if (it->second[2].s > (target_s - 8.0) && it->second[2].s < (target_s + 8.0)) {
				  cost = SAFETY;
			  }
		  }
		  /*
		  // check for efficiency
		  // check for vehicles in front of Ego
		  if (it->second[2].s > target_s) {
			  // check for vehicles in same lane as Ego's target lane
			  if ((it->second[0].lane + lane_change) == target_lane && EFFICIENCY/it->second[0].get_v() > cost) {
				  cost = EFFICIENCY/it->second[0].get_v();
			  }
		  }
		  */
	  }
	  // proceed to lane change by deducting cost by one
	  if (state == "LCL" || state == "LCR") cost -= 1.0;
	  // add minor cost to making any lane change, so keeping lane is preferred if there is no slow traffic
	  //cost += EFFICIENCY*0.01;
	  //std::cout << "F-" << state << " cost:" << cost << std::endl;
  }
  return cost;
}

map<string, float> get_helper_data(const Vehicle &vehicle, 
                                   const vector<Vehicle> &trajectory, 
                                   const map<int, vector<Vehicle>> &predictions) {
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or 
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory.
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help 
  //   differentiate between planning and executing a lane change in the 
  //   cost functions.
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else {
    intended_lane = trajectory_last.lane;
  }

  float distance_to_goal = vehicle.goal_s - trajectory_last.s;
  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
    
  return trajectory_data;
}