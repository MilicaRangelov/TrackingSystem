#pragma once
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class KalmanFilter
{

private:

	int dim_x;
	int dim_y;

	MatrixXd x_prior;
	MatrixXd p_prior;

	MatrixXd x_post;
	MatrixXd p_post;

	MatrixXd B; //adaption matrix
	MatrixXd u; //control variable matrix

	MatrixXd H; //adaption matrix
	MatrixXd K; //kalman gain matrix

	MatrixXd Y; //measurement input
	MatrixXd C; //adaption matrix
	MatrixXd Ykm; //measurement 

	MatrixXd I; //identity matrix

	void predictedState();
	void predictProcessCovarianceMatrix();

	void KalmanGain();
	void theNewObservation();
	void currentState();
	void updateProcessCovarianceMatrix();

public:
	MatrixXd A; //adaption matrix
	MatrixXd R; //measurement error
	MatrixXd Q; //process noise matrix

	MatrixXd x;
	MatrixXd p;

	KalmanFilter(int dim_x = 4, int dim_y = 2);

	~KalmanFilter()
	{

	}

	void predict(const MatrixXd& x0,const MatrixXd& p0, const MatrixXd& B,const MatrixXd& u,const MatrixXd& A,const MatrixXd& Q);
	void update(const MatrixXd& R,const MatrixXd& Y);
};

