#pragma once
#include <vector>
#include "KalmanFilter.h"
#define dim_x 7
#define dim_y 4

class Object
{
private:

	MatrixXd convert_bbox_to_z(const vector<double>& bbox);
	vector<double> convert_x_to_bbox(const MatrixXd& x, double score = 0);

public:
	int object_id;
	KalmanFilter kf;
	int hits;
	int age;
	int hit_streak;
	int time_since_update;


	Object(MatrixXd detect, int obj_id);
	~Object();

	void predict();
	void update(const MatrixXd& bbox);
	MatrixXd get_state();
	vector<double> state();
};

