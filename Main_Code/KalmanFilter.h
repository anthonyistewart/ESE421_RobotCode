#ifndef _KalmanFilter_h_
#define _KalmanFilter_h_

#include <BasicLinearAlgebra.h>

using namespace BLA;

class KalmanFilter{
  public:
    KalmanFilter();

    Matrix<3,1, Array<3,1,double>> prediction(Matrix<2,1, Array<2,1,double>> u, Matrix<2,1, Array<2,1,double>> z_k, double dt);
    Matrix<3,1, Array<3,1,double>> predictionNoCamera(Matrix<2,1, Array<2,1,double>> u, double dt);
    Matrix<2,1, Array<2,1,double>> _hFunction(Matrix<3,1, Array<3,1,double>> x_hat_prime);
    void setQ(Matrix<3,3, Array<3,3,double>> Q_NEW);
    void setR(Matrix<3,3, Array<3,3,double>> R_NEW);
    void setRobotLength(double L);
  
  private:
    double L; //Robot Length
    Matrix<3,3,Array<3,3,double>> I; //Identity Matrix
     
    Matrix<3,3, Array<3,3,double>> Q;
    Matrix<3,3, Array<3,3,double>> R;
    Matrix<3,3, Array<3,3,double>> K_k;

    Matrix <3,3, Array<3,3,double>> A_k;
    Matrix <3,3, Array<3,3,double>> H_k;

    Matrix<3,1,Array<3,1,double>> x_hat;  // Best estimate of the X vector given by the Kalman Filter
    Matrix<3,1, Array<3,1,double>> x_hat_prime;  // The X vector predicted by the Kalman Filter
    Matrix<3,1, Array<3,1,double>> x_hat_last;  // The previous X vector predicted by the Kalman Filter
    
    Matrix<3,3, Array<3,3,double>> P;  // Error Covariance Matrix
    Matrix<3,3, Array<3,3,double>> P_prime;  // Error Covariance Matrix predicted by Kalman Filter
    Matrix<3,3, Array<3,3,double>> P_last;  // previous Error Covariance Matrix predicted by Kalman Filter

    
    void _calculateAMatrix(Matrix<3,1, Array<3,1,double>> x_k, Matrix<2,1, Array<2,1,double>> u, double dt);
    void _calculateHMatrix(Matrix<3,1, Array<3,1,double>> x_k, Matrix<2,1, Array<2,1,double>> z_k, Matrix<2,1, Array<2,1,double>> u, double dt);
};

#endif
