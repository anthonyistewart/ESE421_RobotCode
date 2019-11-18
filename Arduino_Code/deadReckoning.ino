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
int tgtCone;

// servo
Servo steeringServo;
#define servoPin 7 // pin for servo signal
#define SERVOPOT A7
#define SERVOSCALEUS 10
#define SERVOMAXUS 400
#define SERVOMAXDEG 25
double servoAngleDeg;

// motor pins
#define motorFwdPin 8 // HIGH for FWD; LOW for REV
#define motorRevPin 9 // LOW for FWD; HIGH for REV
#define motorLPWMPin 10 // Left Motor Speed Control
#define motorRPWMPin 11 // Right Motor Speed Control
int motorLPWM;
int motorRPWM;

// IMU
#define LSM9DS1_SCK 52
#define LSM9DS1_MISO 50
#define LSM9DS1_MOSI 51
#define LSM9DS1_XGCS 49
#define LSM9DS1_MCS 47
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);


// ping sensor on the front
#define FpingTrigPin 22 // ping sensor trigger pin (output from Arduino)
#define FpingEchoPin 24 // ping sensor echo pin (input to Arduino)
#define FpingGrndPin 26 // ping sensor ground pin (use digital pin as ground)

// cone locations
double const sideLength = 10;
double const cone1X = 0;
double const cone1Y = 0;
double const cone2X = sideLength / 2;
double const cone2Y = sideLength / 2 * sqrt(3);
double const cone3X = sideLength;
double const cone3Y = 0;
double const bufferRange = 3;

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
double v_k = 50;


void setup() {
    Serial.begin(115200);

    // front ping sensor
    pinMode(FpingGrndPin,OUTPUT); digitalWrite(FpingGrndPin,LOW);
    pinMode(FpingTrigPin,OUTPUT);
    pinMode(FpingEchoPin,INPUT);
    
    // servo
    steeringServo.attach(servoPin);
    setServoAngle(servoAngleDeg);

    motorLPWM = velocityToPWM(v_k);
    motorRPWM = motorLPWM;

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
    // center wheels w pot
    double frontDist = getFPingDistanceCM();
    if (frontDist < 5.5) {
          wheelTurnBias = analogRead(SERVOPOT); 
          setServoAngle(0.0);
          currAngle = 0.0;
          tgtCone = 2;
          reinitializeKalman();
          return;
    }
    // IMU setup
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    
    // heading hold
    dt = (micros()-microlast) * 0.001; // convert to sec
    
    currAngle += -(g.gyro.z - rb) * dt;
    
    servoAngleDeg = constrain(KP * (currAngle - desiredAngle), -SERVOMAXDEG, SERVOMAXDEG);
    setServoAngle(servoAngleDeg);
  
    digitalWrite(motorFwdPin, HIGH);
    digitalWrite(motorRevPin,LOW);
    analogWrite(motorLPWMPin,motorLPWM);
    analogWrite(motorRPWMPin,motorRPWM);

    // Kalman filter
    x_k_hat_prime = x_k_hat + v_k * dt; // prediction of running sum w/o measurement
    P_k_prime = P_k + Q; // predicted error
    K_k = P_k_prime / (P_k_prime + R); // filter gain
    z_k = v_k * cos(currAngle) * dt; // fake measurement
    x_k_hat = x_k_hat_prime + K_k * (z_k - x_k_hat_prime); // best estimate w/ measurement
    P_k = P_k_prime * (1 - K_k); // variance of error
    // if we think we've reached a cone, turn 60 deg to the right and head towards next cone
    if (tgtCone == 2 && x_k_hat > cone2X - bufferRange|| x_k_hat < cone2X + bufferRange) {
      tgtCone = 3; 
    } else if (tgtCone == 3 && x_k_hat > cone3X - bufferRange|| x_k_hat < cone3X + bufferRange) {
      tgtCone = 1;
    } else if (tgtCone == 1 && x_k_hat > cone1X - bufferRange|| x_k_hat < cone1X + bufferRange) {
      tgtCone = 2;
    }
    Serial.println(x_k_hat_prime);
    microlast = micros();
}

// helper func to setServoAngle
void setServoAngle(double sDeg) {
    double ServoCenter_us = 800 + wheelTurnBias;
    double ServoScale_us = 8.0;    // micro-seconds per degree
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

double getFPingDistanceCM() {
    const long timeout_us = 3000;
    digitalWrite(FpingTrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(FpingTrigPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(FpingTrigPin, LOW);
    unsigned long echo_time;
    echo_time = pulseIn(FpingEchoPin, HIGH, timeout_us);
    if (echo_time == 0) {
      echo_time = timeout_us;
    }
    double pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
    return pingDistanceCM;
}

void reinitializeKalman() {
    x_k_hat = 0; // best estimate w/0 measurement 
    x_k_hat_prime = 0; // prediction of position w measurement
    x_k = 0; // x position
    z_k = 0; // x_k + v_k
    P_k = 0; // variance of error
    P_k_prime = 0; // predicted error
    K_k = 0; // Kalman gain
}

// vel= cm/s
int velocityToPWM(double v) {
  double a = 1.985 * 0.00001;
  double b = -2.838 * 0.001;
  double c = 0.1479;
  double d = -1.729;
  double e = 50.997;
  double y = (a * pow(v, 4)) + (b * pow(v, 3)) + (c * pow(v, 2)) + (d * v) + e;
  int pwm = ceil(y);
  return pwm;
}
