#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
  this->P.Fill(0);
  this->I.Fill(0);
  this->I(0,0) = 1;
  this->I(1,1) = 1;
  this->I(2,2) = 1;
};

// State (u) Matrix is 2x1 and contains velocity and delta
// Measurement (z_k) Matrix is 2x1 and contains velocity and delta
Matrix<3,1> KalmanFilter::prediction(Matrix<2,1> u, Matrix<3,1> z_k, float dt) {
  //Project the state ahead
  this->x_hat_prime(2) = this->x_hat_last(2) + dt*(u(0)/L)*u(1);    //Calculate psi
  this->x_hat_prime(0) = this->x_hat_last(0) + dt*u(0)*cos(this->x_hat_prime(2)); 
  this->x_hat_prime(1) = this->x_hat_last(1) + dt*u(0)*sin(this->x_hat_prime(2)); 
  

  //Project the covariance error ahead
  this->P_prime = (this->A_k * this->P_last * ~this->A_k) + this->Q;

  //Compute the Kalman Gain
  this->K_k = (this->P_prime * ~this->C_k) * (this->C_k * this->P_prime * ~this->C_k + this->R).Inverse();

  //Update estimate with measurement z_k
  this->x_hat = this->x_hat_prime + (this->K_k * (z_k - h(x_hat_prime)));

  //Update the error covariance
  this->P = (this->I - (this->K_k*this->C_k)) * this->P_prime;

  // Store values for next prediction
  this->x_hat_last = this->x_hat;
  this->P_last = this->P;
  
  return this->x_hat;
}

Matrix<3,1> KalmanFilter::predictionNoCamera(Matrix<2,1> u, float dt){
  //Project the state ahead
  this->x_hat_prime(2) = this->x_hat_last(2) + dt*(u(0)/this->L)*u(1);  //Calculate psi
  this->x_hat_prime(0) = this->x_hat_last(0) + dt*u(0)*cos(this->x_hat_prime(2)); //Calculate X
  this->x_hat_prime(1) = this->x_hat_last(1) + dt*u(0)*sin(this->x_hat_prime(2)); //Calculate Y
  

  //Project the covariance error ahead
  this->P_prime = (this->A_k * this->P_last * ~this->A_k) + this->Q;

  // Store values for next prediction
  this->x_hat_last = this->x_hat_prime;
  this->P_last = this->P_prime;
  
  return (this->x_hat_prime);
}

Matrix<3,1> KalmanFilter::h(Matrix<3,1> x_hat_prime){
  return x_hat_prime;
}

void KalmanFilter::setQ(Matrix<3,3> Q_NEW) { this->Q = Q_NEW; }
void KalmanFilter::setR(Matrix<3,3> R_NEW) { this->R = R_NEW; }
void KalmanFilter::setRobotLength(double L) { this->L = L; }
