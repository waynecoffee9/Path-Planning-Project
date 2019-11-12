#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(Vehicle &Ego, const Vehicle &trajectory,
                     map<int, vector<Vehicle>> &predictions, 
                     const string state);

#endif  // COST_H