#include "KalmanFilter.h"

// initialize Kalman filter matrices
KalmanFilter::KalmanFilter() {
  this->P.Fill(0);
  this->I.Fill(0);
  this->I(0, 0) = 1;
  this->I(1, 1) = 1;
  this->I(2, 2) = 1;
  this->x_hat.Fill(0);
  this->x_hat_last.Fill(0);
  this->x_hat_prime.Fill(0);
};

// State (u) Matrix is 2x1 and contains velocity and delta
// Measurement (z_k) Matrix is 2x1 and contains velocity and r from IMU
Matrix<3, 1, Array<3,1,double>> KalmanFilter::prediction(Matrix<2, 1, Array<2,1,double>> u, Matrix<2, 1, Array<2,1,double>> z_k, double dt) {
  //Project the state ahead
  this->x_hat_prime(0) = this->x_hat_last(0) + (dt * u(0) * cos(this->x_hat_last(2))); //Calculate x
  this->x_hat_prime(1) = this->x_hat_last(1) + (dt * u(0) * sin(this->x_hat_last(2))); //Calculate y

  //Project the covariance error ahead
  this->P_prime = (this->A_k * this->P_last * ~this->A_k) + this->Q;

  //Compute the Kalman Gain
  this->K_k = (this->P_prime * ~this->H_k) * (this->H_k * this->P_prime * ~this->H_k + this->R).Inverse();

  //Update estimate with measurement z_k
  //this->x_hat = this->x_hat_prime + (this->K_k * (z_k - _hFunction(x_hat_prime)));

  //Update the error covariance
  this->P = (this->I - (this->K_k * this->H_k)) * this->P_prime;

  // Store values for next prediction
  this->x_hat_last = this->x_hat;
  this->P_last = this->P;

  return this->x_hat;
}

Matrix<3, 1, Array<3,1,double>> KalmanFilter::predictionNoCamera(Matrix<2, 1, Array<2,1,double>> u, double dt) {
  //Project the state ahead
  this->x_hat_prime(2) = (-u(1) * dt) + this->x_hat_last(2); //Calculate psi
  this->x_hat_prime(0) = this->x_hat_last(0) + (dt * u(0) * cos(this->x_hat_last(2) / 180 * PI)); //Calculate x
  this->x_hat_prime(1) = this->x_hat_last(1) + (dt * u(0) * sin(this->x_hat_last(2) / 180 * PI)); //Calculate y

  //Project the covariance error ahead
  this->P_prime = (this->A_k * this->P_last * ~this->A_k) + this->Q;

  // Store values for next prediction
  this->x_hat_last = this->x_hat_prime;
  this->P_last = this->P_prime;

  return (this->x_hat_prime);
}

Matrix<2, 1, Array<2,1,double>> KalmanFilter::_hFunction(Matrix<3, 1, Array<3,1,double>> x_hat_prime) {
  Matrix<2, 1, Array<2,1,double>> z_k_prime;
  return z_k_prime;
}

void KalmanFilter::_calculateAMatrix(Matrix<3, 1, Array<3,1,double>> x_k, Matrix<2, 1, Array<2,1,double>> u, double dt) {
  this->A_k.Fill(0);
  this->A_k(0, 0) = 1;
  this->A_k(1, 1) = 1;
  this->A_k(2, 2) = 1;
  this->A_k(1, 2) = (-1 * u(0) * dt * sin(x_k(2)));
  this->A_k(2, 2) = (u(0) * dt * cos(x_k(2)));
}

void KalmanFilter::_calculateHMatrix(Matrix<3, 1, Array<3,1,double>> x_k, Matrix<2, 1, Array<2,1,double>> z_k, Matrix<2, 1, Array<2,1,double>> u, double dt) {
  this->H_k(0, 0) = -1 * cos(x_k(2));
  this->H_k(0, 1) = -1 * sin(x_k(2));
  this->H_k(0, 2) = (-1 * (z_k(0) - x_k(0)) * sin(x_k(2))) + ((z_k(1) - x_k(1)) * cos(x_k(2)));
  this->H_k(1, 0) = -1 * cos(x_k(2));
  this->H_k(1, 1) = sin(x_k(2));
  this->H_k(1, 2) = (-1 * (z_k(1) - x_k(1)) * sin(x_k(2))) - ((z_k(0) - x_k(0)) * cos(x_k(2)));
}

void KalmanFilter::setQ(Matrix<3, 3, Array<3,3,double>> Q_NEW) {
  this->Q = Q_NEW;
}
void KalmanFilter::setR(Matrix<3, 3, Array<3,3,double>> R_NEW) {
  this->R = R_NEW;
}
void KalmanFilter::setRobotLength(double L) {
  this->L = L;
}
