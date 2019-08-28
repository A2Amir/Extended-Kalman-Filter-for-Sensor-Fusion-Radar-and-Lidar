#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/* Constructor */
FusionEKF::FusionEKF()
: is_initialized_(false)
, previous_timestamp_(0)

  // initializing matrices
, R_laser_(MatrixXd(2, 2))
, R_radar_(MatrixXd(3, 3))
, H_laser_(MatrixXd(2, 4))
, Hj_(MatrixXd(3, 4))
{
  H_laser_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;

  //measurement covariance matrix - laser
  const float var_px = 0.0225;
  const float var_py = 0.0225;
  R_laser_ << var_px, 0, 0, var_py;

  //measurement covariance matrix - radar
  const float var_rho = 0.09;
  const float var_phi = 0.0009;
  const float var_rho_dot = 0.09;
  R_radar_ << var_rho, 0, 0,
              0, var_phi, 0,
              0, 0, var_rho_dot;

  // laser measurement matrix
/*  H_laser_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;

  Hj_ << 1.0, 0.0, 0.0, 0.0,
         1.0, 1.0, 0.0, 0.0,
         1.0, 1.0, 1.0, 1.0;*/

}

/* Destructor */
FusionEKF::~FusionEKF() {}


/*****************************************************************************
** ProcessMeasurement
 * Receive the measurement dat
 * Initialize the KF
 * Predict
 * Update
 */

void FusionEKF::ProcessMeasurement(const MeasurementPackage& mp)
{
  if (!is_initialized_)
  {
    is_initialized_=true;
    Initialize(mp);
    return;
  }
  
  Predict(mp);
  Update(mp);
}

/*****************************************************************************
**  Initialization
******************************************************************************
 * Initialize the state ekf_.x_ with the first measurement.
 * Create the covariance matrix.
 * Remember: you'll need to convert radar from polar to cartesian coordinates.
 */

void FusionEKF::Initialize(const MeasurementPackage& mp)
{
  //first measurement
  cout << "EKF initialise: " << endl;
  auto x = VectorXd(4);

  if (mp.sensor_type_ == MeasurementPackage::RADAR)
  {
    /*  Convert radar from polar to cartesian coordinates and initialize state.
     */
    const double rho = mp.raw_measurements_(0,0); // Range - radial distance from origin
    const double phi = mp.raw_measurements_(1,0); // Bearing - angle between rho and x
    const double rho_dot = mp.raw_measurements_(2,0); // Radial Velocity - change of p (range rate)
    auto pos = tools.PolarToCartesian(rho, phi);
    auto vel = tools.PolarToCartesian(rho_dot, phi);
    x << pos(0,0), pos(1,0), vel(0,0), vel(1,0);
  }
  else if (mp.sensor_type_ == MeasurementPackage::LASER)
  {
    x <<  mp.raw_measurements_(0,0), mp.raw_measurements_(1,0), 0.0, 0.0;
  }

  // intial state and covariance matrix

  auto P = MatrixXd(4, 4);
  P << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1000.0, 0.0,
        0.0, 0.0, 0.0, 1000.0;

  auto Q = MatrixXd(4,4);

  auto F = MatrixXd(4,4);
  F << 1.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 1.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

  previous_timestamp_ = mp.timestamp_;

  ekf_.Init(x, P, F, Q);

}

/*****************************************************************************
**  Prediction
******************************************************************************
 * Update the state transition matrix F according to the new elapsed time.
 - Time is measured in seconds.
 * Update the process noise covariance matrix.
 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
*/

void FusionEKF::Predict(const MeasurementPackage& mp)
{
  const long current_time_stamp = mp.timestamp_;
  const float dt = (current_time_stamp - previous_timestamp_) / 1.0e6;

  if (dt < 0.005f)
  {
    // Second of two (nearly) simultaneous measurements -> skip prediction.
    return;
  }

  previous_timestamp_ = current_time_stamp;

  // Update Process Covariance Matrix (Q)
  const float dt_2 = dt * dt;
  const float dt_3 = dt_2 * dt;
  const float dt_4 = dt_3 * dt;
  const float var_a = 9; // as suggested by the project.

  ekf_.Q_ <<  dt_4/4*var_a, 0, dt_3/2*var_a, 0,
              0, dt_4/4*var_a, 0, dt_3/2*var_a,
              dt_3/2*var_a, 0, dt_2*var_a, 0,
              0, dt_3/2*var_a, 0, dt_2*var_a;

  //Update State Transition matrix (F)
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  ekf_.Predict();
}


/*****************************************************************************/

VectorXd FusionEKF::PredictRadarMeasurement(const VectorXd& x) const
{
  const float px = x(0,0);
  const float py = x(1,0);
  const float vx = x(2,0);
  const float vy = x(3,0);
  const float eps = 1e-5;
  const float rho = sqrt(px * px + py * py);
  const float phi = atan2(py, px);
  const float rho_dot = (px * vx + py * vy) / (eps + rho);
  VectorXd result(3);
  result << rho, phi, rho_dot;
  return result;
}

/*****************************************************************************
**  Update
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
*/

void FusionEKF::Update(const MeasurementPackage& mp)
{
  if (mp.sensor_type_ == MeasurementPackage::RADAR)
  {
    const auto& z = mp.raw_measurements_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);

    // Predict Radar Measurement z_pred
    const VectorXd z_pred = PredictRadarMeasurement(ekf_.x_);
    ekf_.UpdateEKF(z, z_pred, Hj_, R_radar_);
  }
  else
  {
    ekf_.Update(mp.raw_measurements_, H_laser_, R_laser_);
  }
  // print the output
  std::cout << "x_ = " << ekf_.x_ << endl;
  std::cout << "P_ = " << ekf_.P_ << endl;

}
