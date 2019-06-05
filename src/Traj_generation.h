#ifndef TRAJ_GENERATION_H
#define TRAJ_GENERATION_H

#include <math.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <math.h>
// for convenience
using std::string;
using std::vector;
using std::map;


vector<double> JMT(vector<double> &start, vector<double> &end, double T); //T is timing in the future ;

//define some cost functions for trajectory generation





#endif
