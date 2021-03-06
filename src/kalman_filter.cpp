#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}


void KalmanFilter::UpdateMatrix( const VectorXd &y ) {
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();

    MatrixXd K =  P_ * Ht * Si;

    x_ = x_ + (K * y);

    auto ssize = x_.size();
    MatrixXd I = MatrixXd::Identity( ssize, ssize );
    P_ = (I - K * H_) * P_;
}


void KalmanFilter::Update(const VectorXd &z) {
    VectorXd y = z - H_ * x_;

    UpdateMatrix( y );
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    float phi = atan2(x_(1), x_(0));
    float rho_dot = 0.0;
    if (fabs(rho) < 0.0001) {
        rho_dot += 0.001;
    } else {
        rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
    }
    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;

    VectorXd y = z - z_pred;
	
    /*
     * atan2() returns values between -pi and pi. When calculating phi in y = z - h(x) for radar measurements,
     * the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi.
     * The Kalman filter is expecting small angle values between the range -pi and pi.
     * HINT: when working in radians, you can add 2\pi2π or subtract 2\pi2π until the angle is within the desired range.
     */

    if ( y(1) > M_PI ){
        y(1) -= 2*M_PI;
    } else if ( y(1) < -M_PI ) {
        y(1) += 2*M_PI;
    }

    UpdateMatrix( y );
}
