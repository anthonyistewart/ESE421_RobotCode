#include "math.h"
#include "definitions.h"
#include "KalmanFilter.h"
#include <BasicLinearAlgebra.h>

double velocity = 50.0;

// Kalman Filter
Matrix<3, 3> Q = {0.01, 0, 0,
                  0, 0.01, 0,
                  0, 0, 0.00001
                 };

Matrix<2, 2> R = {0.001, 0,
                  0, 0.001
                 };
KalmanFilter kf = KalmanFilter();

//Cone Positions in cm
int coneX[] = {2000, 85, 60};
int coneY[] = {2000 , 40, 60};
const double minDist = 100;
int current_cone = 0;

void setup() {
  // Enable Serial Communications
  Serial.begin(115200);

  //Initialize Kalman Filter
  kf.setQ(Q);
  kf.setR(R);
  kf.setRobotLength(16);
}

void loop() {
  static unsigned long prevTime;
  double delayTsec = 0.01;
  delay(1000*delayTsec);
  
  // Kalman Filter Calculations
  double dt = ((micros() - prevTime) * 0.000001);
  prevTime = micros();

  Matrix<2> u = {velocity, r_imu};
  Matrix<2> z_k = {0, 0};
  Matrix<3> x_k = kf.prediction(u, z_k, dt);

  // Check to see if we're close to the cone, stop the robot
  if (distToCone(x_k, current_cone) <= minDist) {
    while (true) {
      delay(1);
    }
  }

  double desiredHeading = angleToCone(x_k, current_cone);
  double error = (desiredHeading - x_k(2));

  Serial.print(x_k(0));
  Serial.print(",");
  Serial.print(x_k(1));
  Serial.print(",");
  Serial.println(x_k(2));

  // Proportional Feedback
  servoAngleDeg = K_psi * error;
}

double angleToCone(Matrix<3> x_k, int cone) {
  double beta = 90.0 - ((180 / PI) * atan((x_k(0) - coneX[cone]) / (x_k(1) - coneY[cone])));
  return beta;
}

double distToCone(Matrix<3> x_k, int cone) {
  double dist = sqrt(sq(x_k(0) - coneX[cone]) + sq((x_k(1) - coneY[cone])));
  return dist;
}
