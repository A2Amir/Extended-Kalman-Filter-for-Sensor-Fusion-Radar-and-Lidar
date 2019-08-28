#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
typedef Eigen::Matrix<double, 2, 1>  Vector2f;


//namespace Tools {
class Tools {
public:
	/*Constructor*/
	Tools();
	/*Destructor*/
	virtual ~Tools();

	/* A helper method to calculate RMSE */
	VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

	/*A helper method to calculate Jacobians */
	MatrixXd CalculateJacobian(const VectorXd& x_state);

	/* A helper method to transform Polar coordinates to Cartesian */
	Vector2f PolarToCartesian(double rho, double phi);
};

#endif /* TOOLS_H_ */
