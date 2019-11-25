#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
  this->P.Fill(0);
  this->I.Fill(0);
  this->I(0, 0) = 1;
  this->I(1, 1) = 1;
  this->I(2, 2) = 1;
  this->x_hat.Fill(0);
  Serial.print("X:");
  Serial.print(this->x_hat(0));
  Serial.print(" ,Y:");
  Serial.println(this->x_hat(1));
};

// State (u) Matrix is 2x1 and contains velocity and delta
// Measurement (z_k) Matrix is 2x1 and contains velocity and r from IMU
Matrix<3, 1> KalmanFilter::prediction(Matrix<2, 1> u, Matrix<2, 1> z_k, double dt) {
  //Project the state ahead
  this->x_hat_prime(2) = _correctAngle((u(1) * dt) + this->x_hat_last(2)); //Calculate psi
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

Matrix<3, 1> KalmanFilter::predictionNoCamera(Matrix<2, 1> u, double dt) {
  //Project the state ahead
  //this->x_hat_prime(2) = _correctAngle((u(1)*dt) + this->x_hat_last(2)); //Calculate psi
  this->x_hat_prime(2) = (u(1) * dt) + this->x_hat_last(2); //Calculate psi
  this->x_hat_prime(0) = this->x_hat_last(0) + (dt * u(0) * cos(this->x_hat_last(2))); //Calculate x
  this->x_hat_prime(1) = this->x_hat_last(1) + (dt * u(0) * sin(this->x_hat_last(2))); //Calculate y


  //Project the covariance error ahead
  this->P_prime = (this->A_k * this->P_last * ~this->A_k) + this->Q;

  // Store values for next prediction
  this->x_hat_last = this->x_hat_prime;
  this->P_last = this->P_prime;

  return (this->x_hat_prime);
}

Matrix<2, 1> KalmanFilter::_hFunction(Matrix<3, 1> x_hat_prime) {
  Matrix<2, 1> z_k_prime;

  return z_k_prime;
}

void KalmanFilter::_calculateAMatrix(Matrix<3, 1> x_k, Matrix<2, 1> u, double dt) {
  this->A_k.Fill(0);
  this->A_k(0, 0) = 1;
  this->A_k(1, 1) = 1;
  this->A_k(2, 2) = 1;

  this->A_k(1, 2) = (-1 * u(0) * dt * sin(x_k(2)));
  this->A_k(2, 2) = (u(0) * dt * cos(x_k(2)));
}

void KalmanFilter::_calculateHMatrix(Matrix<3, 1> x_k, Matrix<2, 1> z_k, Matrix<2, 1> u, double dt) {
  this->H_k(0, 0) = -1 * cos(x_k(2));
  this->H_k(0, 1) = -1 * sin(x_k(2));
  this->H_k(0, 2) = (-1 * (z_k(0) - x_k(0)) * sin(x_k(2))) + ((z_k(1) - x_k(1)) * cos(x_k(2)));

  this->H_k(1, 0) = -1 * cos(x_k(2));
  this->H_k(1, 1) = sin(x_k(2));
  this->H_k(1, 2) = (-1 * (z_k(1) - x_k(1)) * sin(x_k(2))) - ((z_k(0) - x_k(0)) * cos(x_k(2)));
}

double KalmanFilter::_correctAngle(double angle) {
  Serial.print("Old Angle:");
  Serial.print(angle);
  double new_angle;
  if (angle > 0) {
    new_angle = (angle * -1) + 360;
  }
  if (angle < 0) {
    new_angle = (angle * -1);
  }
  Serial.print(", New Angle:");
  Serial.println(new_angle);
  return new_angle;
}

void KalmanFilter::setQ(Matrix<3, 3> Q_NEW) {
  this->Q = Q_NEW;
}
void KalmanFilter::setR(Matrix<3, 3> R_NEW) {
  this->R = R_NEW;
}
void KalmanFilter::setRobotLength(double L) {
  this->L = L;
}
