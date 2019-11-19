#include <Adafruit_LSM9DS1.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

// car variables
double const velWithPWM100 = 1.40 / 2.95; // car went 1.4 m in 2.95 seconds at PWM 100;
double dt;
unsigned long microlast;
double rb;
double currAngle;
double KP = -4.5;
double desiredAngle = 0.0;
double wheelTurnBias;

// servo
Servo steeringServo;
#define servoPin 7
#define SERVOPOT A7
#define SERVOMAXDEG 25
double servoAngleDeg;

// motor pins
#define motorFwdPin 8 // HIGH for FWD; LOW for REV
#define motorRevPin 9 // LOW for FWD; HIGH for REV
#define motorLPWMPin 10 // Left Motor Speed Control
#define motorRPWMPin 11 // Right Motor Speed Control
byte motorLPWM=150;
byte motorRPWM=motorLPWM;

// IMU
#define LSM9DS1_SCK 52
#define LSM9DS1_MISO 50
#define LSM9DS1_MOSI 51
#define LSM9DS1_XGCS 49
#define LSM9DS1_MCS 47
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

// cone locations
double const cone1X = 0;
double const cone1Y = 0;
double const cone2X = 0;
double const cone2Y = 5;
double const cone3X = 5;
double const cone3Y = 0;
double const bufferRange = 0.5;

// Kalman variables
double x_k_hat = 0; // best estimate w/0 measurement 
double x_k_hat_prime = 0; // prediction of position w measurement
double x_k = 0; // x position
double z_k = 0; // x_k + v_k
double P_k = 0; // variance of error
double P_k_prime = 0; // predicted error
double K_k = 0; // Kalman gain
// arbitrarily chosen Q and R values
double Q = 1; // cov of pos and orientation disturbances
double R = 1; // cov of image-plane location noise
double v_k = motorLPWM * velWithPWM100;

void setup() {
    Serial.begin(115200);
    
    // servo
    steeringServo.attach(servoPin);
    setServoAngle(servoAngleDeg);

    // motors
    pinMode(motorFwdPin,OUTPUT); digitalWrite(motorFwdPin,HIGH);
    pinMode(motorRevPin,OUTPUT); digitalWrite(motorRevPin,LOW);
    pinMode(motorLPWMPin,OUTPUT); analogWrite(motorLPWMPin,motorLPWM);
    pinMode(motorRPWMPin,OUTPUT); analogWrite(motorRPWMPin,motorRPWM);

    // IMU
    lsm.begin();
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    rb = findRB();
}

void loop() {
    // IMU setup
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    
    // heading hold
    dt = micros()-microlast;
    currAngle += -(g.gyro.z - rb) * dt * 0.000001; // get time in seconds
    
    servoAngleDeg = constrain(KP * (currAngle - desiredAngle), -SERVOMAXDEG, SERVOMAXDEG);
    setServoAngle(servoAngleDeg);
  
    // switch to wall following state if wall is detected
    microlast = micros();
    setServoAngle(servoAngleDeg);
    digitalWrite(motorFwdPin, HIGH);
    digitalWrite(motorRevPin,LOW);
    analogWrite(motorLPWMPin,motorLPWM);
    analogWrite(motorRPWMPin,motorRPWM);
    
    // Kalman filter
    x_k_hat_prime = x_k_hat + v_k * dt; // prediction of running sum w/o measurement
    P_k_prime = P_k + Q; // predicted error
    K_k = P_k_prime / (P_k_prime + R); // filter gain
    x_k_hat = x_k_hat_prime + K_k * (z_k - x_k_hat_prime); // best estimate w/ measurement
    P_k = P_k_prime * (1 - K_k); // variance of error
    // if we think we've reached a cone, turn right and head towards next cone
    if (x_k_hat_prime > cone1X - bufferRange|| x_k_hat_prime < cone1X + bufferRange||
        x_k_hat_prime > cone2X - bufferRange|| x_k_hat_prime < cone2X + bufferRange||
        x_k_hat_prime > cone3X - bufferRange|| x_k_hat_prime < cone3X + bufferRange) {
          desiredAngle = 60;
    }
}

// helper func to setServoAngle
void setServoAngle(double sDeg) {
    double ServoCenter_us = wheelTurnBias;
    double ServoScale_us = 10.0;    // micro-seconds per degree
    double t_us = constrain(ServoCenter_us + ServoScale_us * sDeg, ServoCenter_us-200, ServoCenter_us+200);
    steeringServo.writeMicroseconds(t_us);
}

// finding internal gyro bias
double findRB() {
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    // take 10 readings
    double sum = 0.0;
    for (int i = 0; i < 10; i++) {
      sum += g.gyro.z;
    }
    rb = sum / 10.0;  
}
