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

const float SAFETY = pow(10, 9);
const float EFFICIENCY = pow(10, 5);

float calculate_cost(Vehicle &Ego, const Vehicle &trajectory,
                     map<int, vector<Vehicle>> &predictions, 
                     const string state) {

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
	  if (del_dist > 12 && closest_vehicle_list[Ego.lane][1] < 40
			&& (target_lane == leading_lane || abs(Ego.lane - leading_lane) == 2)) {
		  cost = float(EFFICIENCY/del_dist);
	  } else cost = EFFICIENCY;
	  
	  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
		  // check for safety: are we going to hit another vehicle?
		  // check for other vehicle lane change (simplified Naive Bayes)
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
			  if (it->second[2].s > (target_s - 10.0) && it->second[2].s < (target_s + 14.0)) {
				  cost = SAFETY;
			  }
		  }
		  // check for other vehicles in the same current lane as Ego.
		  if (it->second[0].lane == Ego.lane) {
			  if (it->second[2].s > (target_s - 8.0) && it->second[2].s < (target_s + 8.0)) {
				  cost = SAFETY;
			  }
		  }
	  }
	  // proceed to lane change by deducting cost by one
	  if (state == "LCL" || state == "LCR") cost -= 1.0;
  }
  return cost;
}
