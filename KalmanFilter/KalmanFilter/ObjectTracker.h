#pragma once
#include <vector>
#include <cmath>
#include "Object.h"
#include "Hungarian.h"
#include <fstream>

#define dim_x 7
#define dim_y 4

class ObjectTracker
{

private:

	vector<Object> objects;

	int min_hits;
	int max_age;
	double trash_hold;

	void get_minimum_cost(const MatrixXd& trks,const vector<MatrixXd>& detect, vector<Position>& matched, vector<int>& unmatched);
	double iou_batch(const MatrixXd& trks,const MatrixXd& detect);
	string convert_to_string(MatrixXd track);

public:
	int object_id;
	ObjectTracker(int min_hits = 3, int max_age = 3, double trash_hold = 0.3);
	~ObjectTracker();

	vector<vector<double>> Update(const vector<MatrixXd>& detections);
	vector<string> get_status();
	int get_dimensions();

};

