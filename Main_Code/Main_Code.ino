#include "math.h"
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "KalmanFilter.h"
#include <Wire.h>

// Define Robot Status Flags
#define HALT 0
#define SETUP 1
#define RUNNING 2
#define CALIBRATE 3


// Define Robot Action Flags
#define WALL_FOLLOW 0
#define TURN 1
#define STOP 2
#define DEAD_RECKONING 3

// Define Motor Commands
#define FORWARD 0
#define BACKWARD 1
#define STOP_MOTOR -1

// Define IMU Commands
#define AX 0
#define AY 1
#define AZ 2
#define OMEGAX 3
#define OMEGAY 4
#define OMEGAZ 5

// Define Ping Sensor Directions
#define LEFT_PING 0
#define RIGHT_PING 1
#define FRONT_PING 2

// Define pinouts
#define servoPin 7 // pin for servo signal

#define frontPingTrigPin 22 // ping sensor trigger pin (output from Arduino)
#define frontPingEchoPin 24 // ping sensor echo pin (input to Arduino)
#define frontPingGrndPin 26 // ping sensor ground pin (use digital pin as ground)

#define rightPingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define rightPingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define rightPingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

#define leftPingTrigPin 6 // ping sensor trigger pin (output from Arduino)
#define leftPingEchoPin 5 // ping sensor echo pin (input to Arduino)
#define leftPingGrndPin 0 // ping sensor ground pin (use digital pin as ground)

#define motorFwdPin 8 // HIGH for FWD; LOW for REV
#define motorRevPin 9 // LOW for FWD; HIGH for REV
#define motorLPWMPin 10 // Left Motor Speed Control
#define motorRPWMPin 11 // Right Motor Speed Control

// IMU uses SPI -- here are the pins on the Mega
// (Pins 49 & 47 are user selection)
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega

// Define pins for both pots
#define POT_1 A7
#define POT_2 A8

// Define Constants
const long ping_timeout = 5000; // Timeout for ping sensor
const double desiredDistanceCM = 30.0;  // Desired Distance from wall in CM
const int calibrationTime = 10000;  // Calibration time in milliseconds
const double Kp = 5;  // Proportional Feedback
const double K_psi = 1.5;  // Heading Feeback
const double w_theta = 0.5;  //Theta Filter Cutoff
const double g = 9.81; //Acceleration due to gravity
const double L = 18; //Length of robot in cm
const double velocity = 1; // 100cm/s

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
Matrix<3, 3> Q = {0.001, 0, 0,
                  0, 0.001, 0,
                  0, 0, 0.001
                 };

Matrix<3, 3> R = {0.001, 0, 0,
                  0, 0.001, 0,
                  0, 0, 0.001
                 };
KalmanFilter kf;

byte status_flag = SETUP;
byte action_flag = DEAD_RECKONING;

Servo steeringServo;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

//I2C Communication
const int RECEIVE_REGISTER_SIZE = 8;
const int SEND_REGISTER_SIZE = 8;
float receive_registers[RECEIVE_REGISTER_SIZE];
float send_registers[SEND_REGISTER_SIZE];
int current_send_register = 3;

const int STOP_COMMAND = 100;
const int TURN_COMMAND = 101;
const int DEADRECK_COMMAND = 102;

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
  static unsigned long prevTime_heading;
  static unsigned long prevTime_theta;
  static double dt_kalman;
  static unsigned long prevTime_kalman;
  static byte return_state;

  if (status_flag == RUNNING) {
    //Check front ping sensor distance
    double f_dist = getPingDistance(FRONT_PING);

    if (f_dist <= 5) {
      // Enter Calibration Mode
      status_flag = CALIBRATE;
    }
    /*
      Serial.print("Ping Dist. - R: ");
      Serial.print(getPingDistance(RIGHT_PING), DEC);
      Serial.print(", L: ");
      Serial.print(getPingDistance(LEFT_PING), DEC);
      Serial.print(", F: ");
      Serial.println(getPingDistance(FRONT_PING), DEC);
    */

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


    double error;
    if (action_flag == WALL_FOLLOW) {
      // Move Forward
      moveMotor(50, FORWARD);
      // Lost Wall, Stay Straight
      if (getPingDistance(RIGHT_PING) >= 50.0) {
        error = (last_heading - heading_psi);

        // Proportional Feedback
        servoAngleDeg = -K_psi * error;
      }
      // Follow Wall
      else {
        error = (desiredDistanceCM - getPingDistance(RIGHT_PING));

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
      error = (desired_heading - heading_psi);
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
      dt_kalman = ((micros() - prevTime_kalman) * 0.000001);
      moveMotor(velocityToPWM(velocity), FORWARD);
      if (f_dist <= 5) {
        // Enter Calibration Mode
        status_flag = CALIBRATE;
      }
      else {
        Matrix<2, 1> u = {velocity, servoAngleDeg};
        Matrix<3, 1> x_k = kf.predictionNoCamera(u, dt_kalman);
        error = (0 - x_k(2));

        // Proportional Feedback
        servoAngleDeg = -K_psi * error;
        prevTime_kalman = micros();
      }
      // Set steering angle
      servoAngleDeg = constrain(servoAngleDeg, -20.0, 20.0);
      setServoAngle(servoAngleDeg);
    }

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
    if(status_flag == HALT){
      moveMotor(0, STOP_MOTOR);
    }
//    Serial.print("Servo Angle: ");
//    Serial.println(servoAngleDeg);
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

int velocityToPWM(double v) {
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
  Serial.println(command);

  if(command == STOP_COMMAND){
    Serial.println("Received STOP from Pi");
    action_flag = STOP;
  }
  if(command == TURN_COMMAND){
    Serial.println("Received TURN from Pi");
    action_flag = TURN;
  }
  if(command == DEADRECK_COMMAND){
    Serial.println("Received DEADRECK from Pi");
    action_flag = DEAD_RECKONING;
  }
}

void sendData(){
  char data[8];
  dtostrf(send_registers[current_send_register],8, 4, data);  
  Serial.println(data);  
  Wire.write(data);
}
