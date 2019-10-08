#include "math.h"
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

// Define Robot Status Flags
#define SETUP 0
#define RUNNING 1
#define CALIBRATE 2
#define STOP 3

// Define Robot Action Flags
#define WALL_FOLLOW 0
#define TURN 1

// Define Motor Commands
#define FORWARD 0
#define BACKWARD 1
#define STOP 2

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
const int calibrationTime = 10;  // Calibration time in seconds
const double Kp = 5;  // Proportional Feedback
const double K_psi = 1.5;  // Heading Feeback
const double W = 18; // wheel base length in CM

double accelX_bias; //Bias for X Acceleration
double r_bias;  // Bias for Angular Velocity
double servoAngleDeg = 0.0; // Steering angle delta
int servoBias; //Servo Bias for calibration mode

byte status_flag = SETUP;
byte action_flag = WALL_FOLLOW;

Servo steeringServo;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

void setup() {
  // Enable Serial Communications
  Serial.begin(115200);

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
  unsigned long prevTime = millis();
  double gyroZ;
  double accelX;
  double numSamples;
  while ((millis() - prevTime) < calibrationTime * 1000) {
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    gyroZ += g.gyro.z;
    accelX += a.acceleration.x;
    numSamples++;

    servoBias = analogRead(POT_1);
    setServoAngle(0.0);
  }
  r_bias = gyroZ / numSamples;
  accelX_bias = accelX / numSamples;
  Serial.println("Sensors calibrated!");

  status_flag = RUNNING;
  action_flag = TURN;
}

void loop() {
  static int motor_speed;
  static double dt_heading;
  static double dt_velocity;
  static double heading_psi;
  static double last_heading;
  static double turn_degree = 90;
  static unsigned long prevTime_heading;
  static unsigned long prevTime_velocity;

  if (status_flag == RUNNING) {
    //Check front ping sensor distance
    double f_dist = getPingDistance(FRONT_PING);

    if (f_dist < 10) {
      // Enter Calibration Mode
      status_flag = CALIBRATE;
    }

    // Move Forward
    moveMotor(50, FORWARD);
    Serial.print("Ping Dist. - R: ");
    Serial.print(getPingDistance(RIGHT_PING), DEC);
    Serial.print(", L: ");
    Serial.print(getPingDistance(LEFT_PING), DEC);
    Serial.print(", F: ");
    Serial.println(getPingDistance(FRONT_PING), DEC);

    // Heading Calculation
    dt_heading = ((micros() - prevTime_heading) * 0.000001);
    heading_psi += ((getIMUData(OMEGAZ) - r_bias) * dt_heading);
    prevTime_heading = micros();
    Serial.print("Heading (Ψ): ");
    Serial.println(heading_psi, DEC);

    double error;
    if (action_flag == WALL_FOLLOW) {
      // Lost Wall, Stay Straight
      if (getPingDistance(RIGHT_PING) >= 50.0) {
        error = (last_heading - heading_psi);

        // Proportional Feedback
        servoAngleDeg = -K_psi * error;
      }
      else {
        error = (desiredDistanceCM - getPingDistance(RIGHT_PING));

        // Proportional Feedback
        servoAngleDeg = -Kp * error;
        last_heading = heading_psi;
      }
    }

    if (action_flag == TURN) {

      double desired_heading = turn_degree + last_heading;
      error = (desired_heading - heading_psi);
      if (abs(error) < 0.1) {
        action_flag = WALL_FOLLOW;
        turn_degree = 0;
        last_heading = heading_psi;
        servoAngleDeg = 0;
      }

      else {
        // Proportional Feedback
        servoAngleDeg = -K_psi * error;
      }
    }
    // Set steering angle
    servoAngleDeg = constrain(servoAngleDeg, -20.0, 20.0);
    setServoAngle(servoAngleDeg);
  }
  
  if (status_flag == STOP) {
    while(1){}
  }

  if (status_flag == CALIBRATE) {
    // Stop Motors
    moveMotor(0, STOP);
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
  Serial.println(servoAngleDeg);
}

double getIMUData(byte signalFlag) {
  static double dataIMU[] = {0, 0, 0, 0, 0, 0};
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  dataIMU[AX] = a.acceleration.x;
  dataIMU[AY] = a.acceleration.y;
  dataIMU[AZ] = a.acceleration.z;
  dataIMU[OMEGAX] = g.gyro.x;
  dataIMU[OMEGAY] = g.gyro.y;
  dataIMU[OMEGAZ] = g.gyro.z;

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
  double ServoCenter_us = 800 + servoBias;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     00.0;
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
    case STOP:
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
