#ifndef _KalmanFilter_h_
#define _KalmanFilter_h_

#include <BasicLinearAlgebra.h>

using namespace BLA;

class KalmanFilter{
  public:
    KalmanFilter();

    Matrix<3> prediction(Matrix<2> u, Matrix<2> z_k, double dt);
    Matrix<3> predictionNoCamera(Matrix<2> u, double dt);
    void setQ(Matrix<3,3> Q_NEW);
    void setR(Matrix<3,3> R_NEW);
    void setRobotLength(double L);
  
  private:
    double L; //Robot Length
    Matrix<3,3> I; //Identity Matrix
     
    Matrix<3,3> Q;
    Matrix<3,3> R;
    Matrix<3,3> K;

    Matrix <3,3> A_k;
    Matrix <2,3> H_k;

    Matrix<3> x_hat;  // Best estimate of the X vector given by the Kalman Filter
    Matrix<3> x_hat_prime;  // The X vector predicted by the Kalman Filter
    Matrix<3> x_hat_last;  // The previous X vector predicted by the Kalman Filter
    
    Matrix<3,3> P;  // Error Covariance Matrix
    Matrix<3,3> P_prime;  // Error Covariance Matrix predicted by Kalman Filter
    Matrix<3,3> P_last;  // Previous Error Covariance Matrix

    
    void _calculateAMatrix(Matrix<3> x_k, Matrix<2> u, double dt);
    void _calculateHMatrix(Matrix<3> x_k, Matrix<2> z_k, Matrix<2> u, double dt);
};

#endif
