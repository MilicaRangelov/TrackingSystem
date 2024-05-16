#include "ObjectTracker.h"

void ObjectTracker::get_minimum_cost(const MatrixXd& trks, const vector<MatrixXd>& detect, vector<Position>& matched, vector<int>& unmatched)
{
	//assignes detections to tracked object 
	if (trks.size() == 0) {
		for (int i = 0; i < detect.size(); i++)
			unmatched.push_back(i);
		return;
	}

	int n = trks.rows();
	int m = detect.size();

	MatrixXd cost_matrix;
	cost_matrix.setZero(n, m);
	
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			cost_matrix(i, j) = this->iou_batch(trks.row(i), detect[j]);
		}
	}

	vector<Position> positions;

	if (cost_matrix.cols() > 1) {
		MatrixXd ones;
		ones.setOnes(cost_matrix.rows(), cost_matrix.cols());
		ones = ones - cost_matrix;
		Hungarian hungarian(ones);
		hungarian.substract_rows_columns();
		positions = hungarian.positions();
	}
	else if(cost_matrix.size() > 0){
		Position p;
		p.j = 0;
		double max_value = -1;
		for (int i = 0; i < cost_matrix.rows(); i++) {
			if (cost_matrix(i, 0) > max_value) {
				max_value = cost_matrix(i, 0);
				p.i = i;
			}
		}
		positions.push_back(p);
	}
	for (int i = 0; i < detect.size(); i++) {
		bool contains = false;
		for (int j = 0; j < positions.size(); j++) {
			if (positions[j].j == i) {
				contains = true;
				break;
			}
		}
		if (!contains) {
			unmatched.push_back(i);
		}
	}
	for (int i = 0; i < positions.size(); i++) {
		if (cost_matrix(positions[i].i, positions[i].j) < this->trash_hold) {
			unmatched.push_back(positions[i].j);
		}
		else {
			matched.push_back(positions[i]);
		}
	}
 	
}

double ObjectTracker::iou_batch(const MatrixXd& trks, const MatrixXd& detect)
{
	double x_max = (trks(0,0) > detect(0,0))? trks(0,0) : detect(0,0);
	double x_min= (trks(0, 2) < detect(0, 2)) ? trks(0, 2) : detect(0, 2);
	double y_max = (trks(0, 1) > detect(0, 1)) ? trks(0, 1) : detect(0, 1);
	double y_min = (trks(0, 3) < detect(0, 3)) ? trks(0, 3) : detect(0, 3);
	
	double w = ((x_min - x_max + 1) > 0.0) ? (x_min - x_max + 1) : 0.0;
	double h = ((y_min - y_max + 1) > 0.0) ? (y_min - y_max + 1) : 0.0;

	double value = (w * h) / (( detect(0, 2) - detect(0, 0) + 1) * ( detect(0, 3) - detect(0, 1) + 1) + ( (trks(0, 2) - trks(0, 0) + 1) * ( trks(0, 3) - trks(0, 1) + 1) - (w * h)));
	if (value < this->trash_hold)
		value = 0.0;
	return value;
}

string ObjectTracker::convert_to_string(MatrixXd track)
{
	string value = " ";
	for (int i = 0; i < track.cols(); i++) {
		for (int j = 0; j < track.rows(); j++) {
			value += to_string(track(j, i));
			value += " ";
		}
	}
	return value;
}

vector<string> ObjectTracker::get_status()
{
	vector<string> output;
	for (int i = 0; i < this->objects.size(); i++) {
		string tr = "";
		tr += "ID: " + to_string(objects[i].object_id);
		tr += " ";
		tr += this->convert_to_string(objects[i].get_state());
		output.push_back(tr);
	}

	return output;
}

int ObjectTracker::get_dimensions()
{
	return objects.size();
}

ObjectTracker::ObjectTracker(int min_hits, int max_age, double trash_hold)
{
	this->min_hits = min_hits;
	this->max_age = max_age;
	this->trash_hold = trash_hold;
	this->object_id = 1;
}

ObjectTracker::~ObjectTracker()
{
}

vector<vector<double>> ObjectTracker::Update(const vector<MatrixXd>& detections)
{	
	
	MatrixXd trks;
	trks.setZero(this->objects.size(), 5);
	vector<vector<double>> tracking;

	vector<int> to_del;
	vector<Position> matched;
	vector<int> unmatched_dets;

	for (int i = 0; i < trks.rows(); i++) {
		objects[i].predict();
		MatrixXd pom = objects[i].get_state();
		trks(i, 0) = pom(0, 0);
		trks(i, 1) = pom(0, 1);
		trks(i, 2) = pom(0,2);
		trks(i, 3) = pom(0,3);
		trks(i, 4) = 0;
		// check and add in to_del if needed
		if (isnan(pom(0, 1)) && isnan(pom(0, 1)) && isnan(pom(0, 2)) && isnan(pom(0, 3))) {
			to_del.push_back(i);
		}
	}

	if (to_del.size() > 0) {
		for (int i = 0; i < to_del.size(); i++) {
			objects.erase(this->objects.begin() + to_del[i]);
		}
	}

	//cost_matrix
	this->get_minimum_cost(trks, detections, matched, unmatched_dets);

	for (int i = 0; i < matched.size(); i++) {
		objects[matched[i].i].update(detections[matched[i].j]);
	}

	for (int i = 0; i < unmatched_dets.size(); i++) {

		objects.push_back(Object(detections[unmatched_dets[i]], this->object_id));
		object_id++;
	}

	for (int i = 0; i < this->objects.size(); i++) {
		if (objects[i].time_since_update > this->max_age) {
			objects.erase(this->objects.begin() + i);
		}
	}

	for (int i = 0; i < this->objects.size(); i++) {
		
		tracking.push_back(objects[i].state());
	}

	return tracking;
}
