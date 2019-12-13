#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "math.h"
#include "definitions.h"
#include "KalmanFilter.h"

// Acceleration Bias
double accelX_bias; //Bias for X Acceleration
double accelY_bias; //Bias for Y Acceleration
double accelZ_bias; //Bias for Z Acceleration
double gyroX_bias; //Bias for X Gyro
double gyroY_bias; //Bias for Y Gyro
double gyroZ_bias; //Bias for Z Gyro

double servoAngleDeg = 0.0; // Steering angle delta
int servoBias; //Servo Bias for calibration mode

// Kalman Filter
Matrix<3, 3> Q = {0.01, 0, 0,
                  0, 0.01, 0,
                  0, 0, 0.00001
                 };

Matrix<2, 2> R = {0.001, 0,
                  0, 0.001
                 };
KalmanFilter kf = KalmanFilter();

Servo steeringServo;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

//I2C Communication
const int RECEIVE_REGISTER_SIZE = 8;
const int SEND_REGISTER_SIZE = 8;
float receive_registers[RECEIVE_REGISTER_SIZE];
float send_registers[SEND_REGISTER_SIZE];
int current_send_register = 3;

const int UPDATE_SEND_REGISTER = 11;
const int STOP_COMMAND = 100;
const int TURN_COMMAND = 101;
const int DEADRECK_COMMAND = 102;
const int CAMERA_DATA_AVAILABLE = 103;

//Cone Positions in cm
int coneX[] = {2000, 85, 60};
int coneY[] = {2000 , 40, 60};
const double minDist = 100;
int current_cone = 0;

bool camera_data_available = false;

byte status_flag = INIT;
byte action_flag = DEAD_RECKONING;

void setup() {
  // Enable Serial Communications
  Serial.begin(115200);

  // Initiate i2c bus
  Wire.begin(0x8);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(sendData);

  // Initialize Front Ping Sensor
  pinMode(frontPingGrndPin, OUTPUT); digitalWrite(frontPingGrndPin, LOW);
  pinMode(frontPingTrigPin, OUTPUT);
  pinMode(frontPingEchoPin, INPUT);

  // Initialize Right Ping Sensor
  pinMode(rightPingGrndPin, OUTPUT); digitalWrite(rightPingGrndPin, LOW);
  pinMode(rightPingTrigPin, OUTPUT);
  pinMode(rightPingEchoPin, INPUT);

  // Initialize Left Ping Sensor
  pinMode(leftPingTrigPin, OUTPUT);
  pinMode(leftPingEchoPin, INPUT);

  // Initialize Motor PWM Pins
  pinMode(motorFwdPin, OUTPUT); digitalWrite(motorFwdPin, LOW);
  pinMode(motorRevPin, OUTPUT); digitalWrite(motorRevPin, LOW);
  pinMode(motorLPWMPin, OUTPUT);

  // Initialize Servo
  steeringServo.attach(servoPin);
  setServoAngle(servoAngleDeg);

  //Initialize IMU
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  //Set ranges for sensor
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  Serial.println("Calibrating Sensors...");
  calibrateIMU();
  Serial.println("Sensors calibrated!");

  //Initialize Kalman Filter
  kf.setQ(Q);
  kf.setR(R);
  kf.setRobotLength(L);

  status_flag = RUNNING;
  action_flag = DEAD_RECKONING;
}

void loop() {
  static int motor_speed;
  static double dt_heading;
  static double dt_theta;
  static double heading_psi;
  static double theta_hat;
  static double theta_bias;
  static double last_heading;
  static double turn_degree = 90;
  static double r_imu;
  static unsigned long prevTime_heading;
  static unsigned long prevTime_theta;
  static unsigned long prevTime_kalman;
  static byte return_state;

  if (status_flag == RUNNING) {

    //Check front ping sensor distance
    double f_dist = getPingDistance(FRONT_PING);

    if (f_dist <= 5) {
      // Enter Calibration Mode
      status_flag = CALIBRATE;
    }

    // Heading Calculation
    dt_heading = ((micros() - prevTime_heading) * 0.000001);
    heading_psi += (getIMUData(OMEGAZ) * dt_heading);
    prevTime_heading = micros();
    /*
      Serial.print("Heading (Ψ): ");
      Serial.println(heading_psi, DEC);
    */

    // Theta Hat Calculation
    dt_theta = ((micros() - prevTime_theta) * 0.000001);
    double theta_g = -57.3 * getIMUData(AY) * (1 / g);
    theta_bias = w_theta * (theta_g - theta_hat);
    theta_hat += ((theta_bias + getIMUData(OMEGAX)) * dt_theta);
    prevTime_theta = micros();
    /*
      Serial.print("Theta Hat (θ): ");
      Serial.println(theta_hat, DEC);
    */

    if (action_flag == WALL_FOLLOW) {
      // Move Forward
      moveMotor(50, FORWARD);
      // Lost Wall, Stay Straight
      if (getPingDistance(RIGHT_PING) >= 50.0) {
        double error = (last_heading - heading_psi);

        // Proportional Feedback
        servoAngleDeg = -K_psi * error;
      }
      // Follow Wall
      else {
        double error = (desiredDistanceCM - getPingDistance(RIGHT_PING));

        // Proportional Feedback
        servoAngleDeg = -Kp * error;
        last_heading = heading_psi;
      }
    }

    //EXECUTE TURN
    else if (action_flag == TURN) {
      // Move Forward
      moveMotor(50, FORWARD);

      double desired_heading = turn_degree + last_heading;
      double error = (desired_heading - heading_psi);
      if (abs(error) < 0.1) {
        action_flag = return_state;
        turn_degree = 0;
        last_heading = heading_psi;
        servoAngleDeg = 0;
      }
      else {
        // Proportional Feedback
        servoAngleDeg = -K_psi * error;
      }
    }

    // Stop Robot
    else if (action_flag == STOP) {
      // Stop Motors
      moveMotor(0, STOP_MOTOR);
    }

    // Dead Reckoning
    else if (action_flag == DEAD_RECKONING) {

      // Enter Calibration Mode
      if (f_dist <= 5) {
        status_flag = CALIBRATE;
      }

      else {
        // Kalman Filter Calculations
        double dt_kalman = ((micros() - prevTime_kalman) * 0.000001);
        r_imu = getIMUData(OMEGAZ);
        prevTime_kalman = micros();

        Matrix<2> u = {velocity, r_imu};
        Matrix<3> x_k;
        if(camera_data_available){
          Matrix<2> z_k = {0,0};
          x_k = kf.prediction(u, z_k, dt_kalman);
          camera_data_available = false;
        }
        else{
          x_k = kf.predictionNoCamera(u, dt_kalman);
        }
        
        // Check to see if we're close to the cone, stop the robot
        if (distToCone(x_k, current_cone) <= minDist) {
          moveMotor(0, STOP_MOTOR);
          while (true) {
            delay(1);
          }
        }

        double desiredHeading = angleToCone(x_k, current_cone);
        double error = (desiredHeading - x_k(2));

        Serial.print("X: ");
        Serial.print(x_k(0));
        Serial.print(",Y: ");
        Serial.print(x_k(1));
        Serial.print(",PSI: ");
        Serial.println(x_k(2));

        // Proportional Feedback
        servoAngleDeg = K_psi * error;

      }
      // Move Motor
      moveMotor(velocityToPWM(velocity), FORWARD);

      // Set steering angle
      servoAngleDeg = constrain(servoAngleDeg, -20.0, 20.0);
      setServoAngle(servoAngleDeg);
    }

    // Calibration mode for the robot
    if (status_flag == CALIBRATE) {
      // Stop Motors
      moveMotor(0, STOP_MOTOR);
      double dist = getPingDistance(FRONT_PING);
      do {
        servoBias = analogRead(POT_1);
        setServoAngle(0.0);
        Serial.print("Servo Bias: ");
        Serial.println(servoBias);
        dist = getPingDistance(FRONT_PING);
      } while (dist < 10);

      status_flag = RUNNING;
    }
  }
}

double getIMUData(byte signalFlag) {
  static double dataIMU[] = {0, 0, 0, 0, 0, 0};
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  dataIMU[AX] = a.acceleration.x - accelX_bias;
  dataIMU[AY] = a.acceleration.y - accelY_bias;
  dataIMU[AZ] = a.acceleration.z - accelZ_bias;
  dataIMU[OMEGAX] = g.gyro.x - gyroX_bias;
  dataIMU[OMEGAY] = g.gyro.y - gyroY_bias;
  dataIMU[OMEGAZ] = g.gyro.z - gyroZ_bias;
  return dataIMU[signalFlag];
}

double getPingDistance(byte pingDir) {
  //LEFT - RIGHT - FRONT
  static double pingDistanceCM[] = {0.0, 0.0, 0.0};
  byte trigPins[] = {leftPingTrigPin, rightPingTrigPin, frontPingTrigPin};
  byte echoPins[] = {leftPingEchoPin, rightPingEchoPin, frontPingEchoPin};
  byte grndPins[] = {leftPingGrndPin, rightPingGrndPin, frontPingGrndPin};

  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = ping_timeout;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(trigPins[pingDir], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[pingDir], HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPins[pingDir], LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(echoPins[pingDir], HIGH, timeout_us);
  if (echo_time == 0)
  {
    echo_time = timeout_us;
  }
  //
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  return constrain(0.017 * echo_time, 5.0, 50.0);
}

void setServoAngle(double sDeg)
{
  //
  //  Update ServoCenter_us as Required for installation bias
  //  CAREFUL: make small increments (~100) to iterate
  //  100us is about 20deg (higher values --> more right steering)
  //  wrong ServoCenter values can damage servo
  //
  double ServoCenter_us = 800 + servoBias;
  double ServoScale_us = 8.0;    // micro-seconds per degree
  //
  //  NEVER send a servo command without constraining servo motion!
  //  -->large servo excursions could damage hardware or servo<--
  //
  double t_us = constrain(ServoCenter_us + ServoScale_us * sDeg, ServoCenter_us - 150, ServoCenter_us + 150);
  steeringServo.writeMicroseconds(t_us);
}

//Motor Function takes in speed integer from 0-100
void moveMotor(int motor_speed, byte direction) {
  switch (direction) {
    case FORWARD:
      digitalWrite(motorFwdPin, HIGH);
      digitalWrite(motorRevPin, LOW);
      break;
    case BACKWARD:
      digitalWrite(motorFwdPin, LOW);
      digitalWrite(motorRevPin, HIGH);
      break;
    case STOP_MOTOR:
      digitalWrite(motorFwdPin, LOW);
      digitalWrite(motorRevPin, LOW);
      break;
    default:
      break;
  }

  byte motorPWM = map(motor_speed, 0, 100, 0, 255);
  analogWrite(motorLPWMPin, motorPWM);
  analogWrite(motorRPWMPin, motorPWM);
}

void calibrateIMU() {
  double gyroX;
  double gyroY;
  double gyroZ;
  double accelX;
  double accelY;
  double accelZ;
  int numSamples;
  unsigned long prevTime = millis();
  unsigned long elapsedTime;
  do {
    elapsedTime = millis() - prevTime;
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    gyroX += g.gyro.x;
    gyroY += g.gyro.y;
    gyroZ += g.gyro.z;

    accelX += a.acceleration.x;
    accelY += a.acceleration.y;
    accelZ += a.acceleration.z;
    numSamples++;

    servoBias = analogRead(POT_1);
    setServoAngle(0.0);
  } while (elapsedTime < calibrationTime);
  gyroX_bias = gyroX / numSamples;
  gyroY_bias = gyroY / numSamples;
  gyroZ_bias = gyroZ / numSamples;

  accelX_bias = accelX / numSamples;
  accelY_bias = accelY / numSamples;
  accelZ_bias = accelZ / numSamples;
}

// Takes in a value for velocity in cm/s and returns a value in pwm
int velocityToPWM(double v_cm_s) {
  double v = v_cm_s / 100.0;
  double a = 1.985 * 0.00001;
  double b = -2.838 * 0.001;
  double c = 0.1479;
  double d = -1.729;
  double e = 50.997;
  double y = (a * pow(v, 4)) + (b * pow(v, 3)) + (c * pow(v, 2)) + (d * v) + e;
  return ceil(y);
}

void receiveEvent(int howMany) {
  String full_datastring = "";

  while (Wire.available()) {
    char c = Wire.read();
    full_datastring = full_datastring + c;
  }

  byte command = full_datastring.charAt(0);

  if (command == STOP_COMMAND) {
    Serial.println("Received STOP from Pi");
    action_flag = STOP;
  }
  if (command == TURN_COMMAND) {
    Serial.println("Received TURN from Pi");
    action_flag = TURN;
  }
  if (command == DEADRECK_COMMAND) {
    Serial.println("Received DEADRECK from Pi");
    action_flag = DEAD_RECKONING;
  }
  if (command == CAMERA_DATA_AVAILABLE) {
    Serial.println("Received CAMERA_DATA_AVAILABLE from Pi");
    camera_data_available = true;
  }
  if (command == UPDATE_SEND_REGISTER) {
    int data = full_datastring.substring(1).toInt();
    current_send_register = data;
    Serial.println("updating send register");
    Serial.println(current_send_register);
  }
  // If the command is inside the default write register range then write to the write register
  if (command >= 0 && command < RECEIVE_REGISTER_SIZE)  {
    // received a float and therefore write to a register
    Serial.println("Received Data Float. Writing to register " + String(command));
    float data = full_datastring.substring(1).toFloat();
    Serial.println(data);
    receive_registers[command] = data;
  }
}

void sendData() {
  char data[8];
  dtostrf(send_registers[current_send_register], 8, 4, data);
  // Serial.println(data);
  Wire.write(data);
}

double angleToCone(Matrix<3> x_k, int cone) {
  double beta = 90.0 - ((180 / PI) * atan((x_k(0) - coneX[cone]) / (x_k(1) - coneY[cone])));
  return beta;
}

double distToCone(Matrix<3> x_k, int cone) {
  double dist = sqrt(sq(x_k(0) - coneX[cone]) + sq((x_k(1) - coneY[cone])));
  return dist;
}