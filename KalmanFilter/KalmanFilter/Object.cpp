#include "Object.h"

MatrixXd Object::convert_bbox_to_z(const vector<double>& bbox)
{
	double w = bbox[2] - bbox[0];
	double h = bbox[3] - bbox[1];

	MatrixXd x0;
	x0.setZero(dim_x, 1);
	x0(0, 0) = bbox[0] + w / 2;
	x0(1, 0) = bbox[1] + h / 2;
	x0(2, 0) = w * h;
	x0(3, 0) = w / h;
	return x0;

}

vector<double> Object::convert_x_to_bbox(const MatrixXd& x, double score)
{
	vector<double> bbox;
	double w = sqrt(x(2, 0) * x(3, 0));
	double h = x(2, 0) / w;

	bbox.push_back(x(0, 0) - w / 2);
	bbox.push_back(x(1, 0) - h / 2);
	bbox.push_back(x(0, 0) + w / 2);
	bbox.push_back(x(1, 0) + h / 2);
	bbox.push_back((double)this->object_id);

	return bbox;
}

Object::Object(MatrixXd detect,int obj_id) : kf(KalmanFilter(dim_x, dim_y))
{
	this->object_id = obj_id;
	this->hits = 0;
	this->age = 0;
	this->hit_streak = 0;
	this->time_since_update = 0;

	MatrixXd A;
	A.setOnes(dim_x, dim_x);
	A << 1, 0, 0, 0, 1, 0, 0,
		0, 1, 0, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 0, 1,
		0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 1;

	this->kf.A = A.replicate(1, 1);

	MatrixXd x0;
	vector<double> pom;
	for (int i = 0; i < detect.rows(); i++) {
		for (int j = 0; j < detect.cols(); j++) {
			pom.push_back(detect(i, j));
		}
	}
	
	x0 = this->convert_bbox_to_z(pom);

	this->kf.x = x0.replicate(1, 1);

	MatrixXd p0;
	p0.setIdentity(dim_x, dim_x);
	for (int i = 4; i < dim_x; i++) {
		for (int j = 4; j < dim_x; j++) {
			p0(i, j) *= 1000;
		}
	}
	p0 *= 10;

	this->kf.p = p0.replicate(1, 1);

	MatrixXd R;
	R.setIdentity(dim_x, dim_x);
	for (int i = 2; i < dim_x; i++) {
		for (int j = 2; j < dim_x; j++) {
			R(i, j) *= 10;
		}
	}

	this->kf.R = R.replicate(1, 1);

	MatrixXd Q;
	Q.setIdentity(dim_x, dim_x);
	for (int i = 4; i < dim_x; i++) {
		for (int j = 4; j < dim_x; j++) {
			Q(i, j) *= 0.01;
		}
	}

	this->kf.Q = Q.replicate(1, 1);

}

Object::~Object()
{
	//delete kf;
}

void Object::predict()
{
	MatrixXd A0, B0, Q0, R0, x0, p0, u;
	this->kf.predict(x0, p0, B0, u, A0, Q0);
	this->age++;
	if (this->time_since_update > 0)
		this->hit_streak = 0;
	this->time_since_update++;
}

void Object::update(const MatrixXd& bbox)
{
	MatrixXd R;

	vector<double> pom;
	for (int i = 0; i < bbox.rows(); i++) {
		for (int j = 0; j < bbox.cols(); j++) {
			pom.push_back(bbox(i, j));
		}
	}

	this->kf.update(R,this->convert_bbox_to_z(pom));
	this->hits++;
	this->hit_streak++;
	this->time_since_update = 0;
}

MatrixXd Object::get_state()
{
	MatrixXd state;
	state.setZero(1, 5);

	vector<double> pom = this->convert_x_to_bbox(this->kf.x);
	for (int i = 0; i < 5; i++) {
		state(0, i) = pom[i];
	}
	return state;
}

vector<double> Object::state()
{
	return this->convert_x_to_bbox(this->kf.x);
}

