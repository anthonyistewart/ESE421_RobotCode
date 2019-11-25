#ifndef _KalmanFilter_h_
#define _KalmanFilter_h_

#include <BasicLinearAlgebra.h>

using namespace BLA;

class KalmanFilter{
  public:
    KalmanFilter();

    Matrix<3,1> prediction(Matrix<2,1> u, Matrix<2,1> z_k, double dt);
    Matrix<3,1> predictionNoCamera(Matrix<2,1> u, double dt);
    Matrix<2,1> _hFunction(Matrix<3,1> x_hat_prime);
    void setQ(Matrix<3,3> Q_NEW);
    void setR(Matrix<3,3> R_NEW);
    void setRobotLength(double L);
    double _correctAngle(double angle);
  
  private:
    double L; //Robot Length
    Matrix<3,3> I; //Identity Matrix
     
    Matrix<3,3> Q;
    Matrix<3,3> R;
    Matrix<3,3> K_k;

    Matrix <3,3> A_k;
    Matrix <3,3> H_k;

    Matrix<3,1> x_hat;  // Best estimate of the X vector given by the Kalman Filter
    Matrix<3,1> x_hat_prime;  // The X vector predicted by the Kalman Filter
    Matrix<3,1> x_hat_last;  // The previous X vector predicted by the Kalman Filter
    
    Matrix<3,3> P;  // Error Covariance Matrix
    Matrix<3,3> P_prime;  // Error Covariance Matrix predicted by Kalman Filter
    Matrix<3,3> P_last;  // previous Error Covariance Matrix predicted by Kalman Filter

    
    void _calculateAMatrix(Matrix<3,1> x_k, Matrix<2,1> u, double dt);
    void _calculateHMatrix(Matrix<3,1> x_k, Matrix<2,1> z_k, Matrix<2,1> u, double dt);
};

#endif
