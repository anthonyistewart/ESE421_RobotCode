#include "KalmanFilter.h"

// initialize Kalman filter matrices
KalmanFilter::KalmanFilter() {
  //Initialize Matrices
  this->K.Fill(0);
  this->Q.Fill(0);
  this->R.Fill(0);

  this->P.Fill(0);
  this->P_last.Fill(0);
  this->P_prime.Fill(0);

  this->I = {1,0,0,
             0,1,0,
             0,0,1};

  this->x_hat.Fill(0);
  this->x_hat_last.Fill(0);
  this->x_hat_prime.Fill(0);
};

// State (u) Matrix is 2x1 and contains velocity and delta
// Measurement (z_k) Matrix is 2x1 and contains velocity and r from IMU
Matrix<3> KalmanFilter::prediction(Matrix<2> u, Matrix<2> z_k, double dt) {
  
  //Project the state ahead
  this->x_hat_prime(2) = (-u(1) * dt) + this->x_hat_last(2); //Calculate psi
  this->x_hat_prime(0) = this->x_hat_last(0) + (dt * u(0) * cos(this->x_hat_last(2))); //Calculate x
  this->x_hat_prime(1) = this->x_hat_last(1) + (dt * u(0) * sin(this->x_hat_last(2))); //Calculate y
  //Project the covariance error ahead
  _calculateAMatrix(this->x_hat_prime, u, dt);

  Matrix<3, 3> temp = this->A_k * this->P_last;
  Multiply(temp, ~this->A_k, this->P_prime);
  Add(this->P_prime, this->Q, this->P_prime);

  //Compute the Kalman Gain
  Matrix<2, 3> temp1 =  this->H_k * this->P_prime;
  Matrix<2, 2> temp2 = temp1 * ~this->H_k + this->R;
  Matrix<3, 2> temp3 = ~this->H_k * temp2.Inverse();
  this->K = this->P_prime * temp3;

  //Compute z_k_prime
  Matrix<2> z_k_prime = { ((this->cone(0) - x_hat_prime(0))*cos(x_hat_prime(2))) + ((this->cone(1) - x_hat_prime(1))*sin(x_hat_prime(2))) - this->L,
                          ((this->cone(1) - x_hat_prime(1))*cos(x_hat_prime(2))) - ((this->cone(0) - x_hat_prime(0))*sin(x_hat_prime(2)))};

  //Update estimate with measurement z_k
  this->x_hat = this->x_hat_prime + (this->K * (z_k - z_k_prime));

  //Update the error covariance
  this->P = (this->I - (this->K * this->H_k)) * this->P_prime;

  // Store values for next prediction
  this->x_hat_last = this->x_hat;
  this->P_last = this->P;

  return this->x_hat_prime;
}

// State (u) Matrix is 2x1 and contains velocity and delta
Matrix<3> KalmanFilter::predictionNoCamera(Matrix<2> u, double dt) {
  //Project the state ahead
  this->x_hat_prime(2) = (-u(1) * dt) + this->x_hat_last(2); //Calculate psi
  this->x_hat_prime(0) = this->x_hat_last(0) + (dt * u(0) * cos(this->x_hat_last(2))); //Calculate x
  this->x_hat_prime(1) = this->x_hat_last(1) + (dt * u(0) * sin(this->x_hat_last(2))); //Calculate y

  //Project the covariance error ahead
  _calculateAMatrix(this->x_hat_prime, u, dt);

  Matrix<3, 3> temp = this->A_k * this->P_last;
  Multiply(temp, ~this->A_k, this->P_prime);
  Add(this->P_prime, this->Q, this->P_prime);

  // Store values for next prediction
  this->x_hat_last = this->x_hat_prime;
  this->P_last = this->P_prime;

  return (this->x_hat_prime);
}

void KalmanFilter::_calculateAMatrix(Matrix<3> x_k, Matrix<2> u, double dt) {
  this->A_k(0, 0) = 1;
  this->A_k(1, 1) = 1;
  this->A_k(2, 2) = 1;
  this->A_k(0, 2) = (-1 * u(0) * dt * sin(x_k(2)));
  this->A_k(1, 2) = (u(0) * dt * cos(x_k(2)));
}

void KalmanFilter::_calculateHMatrix(Matrix<3> x_k, Matrix<2> z_k, Matrix<2> u, double dt) {
  this->H_k(0, 0) = -1 * cos(x_k(2));
  this->H_k(0, 1) = -1 * sin(x_k(2));
  this->H_k(0, 2) = (-1 * (z_k(0) - x_k(0)) * sin(x_k(2))) + ((z_k(1) - x_k(1)) * cos(x_k(2)));
  this->H_k(1, 0) = sin(x_k(2));
  this->H_k(1, 1) = -1 * cos(x_k(2));
  this->H_k(1, 2) = (-1 * (z_k(1) - x_k(1)) * sin(x_k(2))) - ((z_k(0) - x_k(0)) * cos(x_k(2)));
}

void KalmanFilter::setQ(Matrix<3, 3> Q_NEW) {
  this->Q = Q_NEW;
}
void KalmanFilter::setR(Matrix<2, 2> R_NEW) {
  this->R = R_NEW;
}
void KalmanFilter::setRobotLength(double L) {
  this->L = L;
}
void KalmanFilter::setCurrentCone(Matrix<2> cone_curr){
  this->cone = cone_curr;
}
