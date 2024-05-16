#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

using std::vector;
using std::string;
using Eigen::MatrixXd;

class ObjectTrackerInterface
{
public:
	ObjectTrackerInterface() = default;
	virtual ~ObjectTrackerInterface() = default;

	virtual vector<vector<double>> Update(vector<MatrixXd> detections) = 0;
	virtual vector<string> get_status() = 0;
};