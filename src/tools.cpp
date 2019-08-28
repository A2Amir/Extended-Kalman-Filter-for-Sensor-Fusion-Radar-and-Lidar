#include <iostream>
#include "tools.h"
#include <cmath>


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

/** RMSE function */
VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
                              const vector<VectorXd>& ground_truth) {
  // check that the estimation vector size equal to ground_truth vector size
  assert(estimations.size() == ground_truth.size());
  // check that the estimation vector size is not zero
  assert(estimations.size() > 0);
  auto n_vectors = estimations.size();
  
  assert(estimations[0].size() == ground_truth[0].size());
  auto dim_vector = estimations[0].size();
  
  //accumulate squared residuals
  VectorXd rmse(dim_vector); rmse.fill(0);
  for (auto i=0; i<n_vectors; ++i){
    //calculate the squared difference of the residuals
    VectorXd sqr_residual = (estimations[i]-ground_truth[i]).array().square();
    //Aggregate the residuals
    rmse += sqr_residual;
  }
  
  //Apply sqrt and average
  rmse = (rmse/n_vectors).array().sqrt();
  
  //Return the result
  return rmse;
}


/** Jacobian Matrix function */
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  MatrixXd Hj(3,4);
  //recover state parameters
  const double px = x_state(0);
  const double py = x_state(1);
  const double vx = x_state(2);
  const double vy = x_state(3);
  
  //pre-compute a set of terms to avoid repeated calculation
  const double c1 = std::max(1.0e-8, px*px+py*py);
  const double c2 = std::max(1.0e-8, sqrt(c1));
  const double c3 = std::max(1.0e-8, (c1*c2));
  
  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
  -(py/c1), (px/c1), 0, 0,
  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  
  return Hj;
  
}

/** Polar to Cartesian function */

Vector2f Tools::PolarToCartesian(double rho, double phi)
{
  auto result = Vector2f();
  result << rho * cos(phi), rho * sin(phi);
  return result;
}
