#include "KalmanFilter.h"


void KalmanFilter::predictedState()
{
	//Xkp = AXk-1 + Buk + wk;
	this->x = this->A * this->x + this->B * this->u;
	this->x_prior = this->x.replicate(1, 1);
}

void KalmanFilter::predictProcessCovarianceMatrix()
{
	//Pkp = A*Pk-1*At + Qk
	this->p = this->p * this->A.transpose();
	this->p = this->A * this->p;
	this->p += this->Q;

	for (int i = 0; i < this->dim_x; i++) {
		for (int j = 0; j < this->dim_x; j++) {

			if (i != j) {
				this->p(i, j) = 0;
			}
		}
	}
	this->p_prior = this->p.replicate(1, 1);
}

void KalmanFilter::KalmanGain()
{
	MatrixXd currEst = this->p * this->H.transpose();
	MatrixXd currEstMea = this->H * currEst;
	currEstMea += this->R;
	this->K = currEst * currEstMea.inverse();
}

void KalmanFilter::theNewObservation()
{
	//Yk = C * Ykm
	this->Y = this->C * this->Ykm;
}

void KalmanFilter::currentState()
{
	MatrixXd pom = this->H * this->x;
	this->x = this->x + (this->K * (this->Y - pom ));

	this->x_post = this->x.replicate(1, 1);
}

void KalmanFilter::updateProcessCovarianceMatrix()
{
	// Pk = (I - K*H) * Pkp
	MatrixXd pom = this->K * this->H;
	pom = this->I - pom;
	this->p = pom * this->p;

	this->p_post = this->p.replicate(1, 1);
}

KalmanFilter::KalmanFilter(int dim_x, int dim_y)
{
	this->dim_x = dim_x;
	this->dim_y = dim_y;

	this->x.setOnes(this->dim_x, 1);
	this->p.setIdentity(this->dim_x, 1);

	this->x_prior = this->x.replicate(1, 1);
	this->p_prior = this->p.replicate(1, 1);

	this->x_post = this->x.replicate(1, 1);
	this->p_post = this->p.replicate(1, 1);

	this->u.setZero(this->dim_y, 1);
	this->A.setIdentity(this->dim_x, this->dim_x);
	this->B.setOnes(this->dim_x, this->dim_y);

	this->H.setIdentity(this->dim_x, dim_x);
	this->K.setZero(this->dim_x, 1);
	this->R.setIdentity(this->dim_x, this->dim_x);
	this->Q.setIdentity(this->dim_x, this->dim_x);

	this->C.setIdentity(this->dim_x, this->dim_x);
	this->Y.setOnes(this->dim_x, 1);

	this->I.setIdentity(this->dim_x, this->dim_x);


}

void KalmanFilter::predict(const MatrixXd& x0, const MatrixXd& p0, const MatrixXd& B, const MatrixXd& u, const MatrixXd& A, const MatrixXd& Q)
{
	// x0 - initial state 
	if (x0.size() != 0) {
		this->x = x0.replicate(1, 1);
	}

	if (p0.size() != 0) {
		this->p = p0.replicate(1, 1);
	}

	if (A.size() != 0) {
		this->A = A.replicate(1, 1);
	}

	if (B.size() != 0) {
		this->B = B.replicate(1, 1);
	}

	if (u.size() != 0) {
		this->u = u.replicate(1, 1);
	}

	if (Q.size() != 0) {
		this->Q = Q.replicate(1, 1);
	}
	this->predictedState();
	this->predictProcessCovarianceMatrix();
}

void KalmanFilter::update(const MatrixXd& R, const MatrixXd& Y)
{
	if (R.size() != 0) {
		this->R = R.replicate(1, 1);
	}
	if (Y.size() != 0) {
		this->Ykm = Y.replicate(1, 1);
	}

	this->KalmanGain();
	this->theNewObservation();
	this->currentState();
	this->updateProcessCovarianceMatrix();

}
