#include "kalman_filter.h"
//#include <math.h>
//#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(
  VectorXd& x_in,
  MatrixXd& P_in,
  MatrixXd& F_in,
  MatrixXd& Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
  I_ = MatrixXd::Identity(P_.rows(), P_.cols());
}

/** Predict the state */
void KalmanFilter::Predict()
{
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

/** Update the state by using Kalman Filter equations */
void KalmanFilter::Update(const VectorXd &z,
                          const MatrixXd& H,
                          const MatrixXd& R)
{
  const VectorXd z_pred = H*x_;
  UpdateEKF(z, z_pred, H, R);
}


/** Update the state by using Extended Kalman Filter equations (with already predicted measurements) */

void KalmanFilter::UpdateEKF(const VectorXd& z,
                             const VectorXd& z_pred,
                             const MatrixXd& H,
                             const MatrixXd& R) {

  /*
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot;
  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot; 
  */

  const VectorXd y = z - z_pred;
  const MatrixXd Ht = H.transpose();
  const MatrixXd S = H * P_ * Ht + R;
  const MatrixXd K = P_ * Ht * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H) * P_;
}