#include <Adafruit_LSM9DS1.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

// motor pins
#define motorFwdPin 8 // HIGH for FWD; LOW for REV
#define motorRevPin 9 // LOW for FWD; HIGH for REV
#define motorLPWMPin 10 // Left Motor Speed Control
#define motorRPWMPin 11 // Right Motor Speed Control
byte motorLPWM=100;
byte motorRPWM=100;

// IMU
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

// ping sensor on the left
#define LpingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define LpingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define LpingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

// ping sensor on the right
#define RpingTrigPin 6 // ping sensor trigger pin (output from Arduino)
#define RpingEchoPin 5 // ping sensor echo pin (input to Arduino)

// front ping sensor
#define FpingTrigPin 22 // ping sensor trigger pin (output from Arduino)
#define FpingEchoPin 24 // ping sensor echo pin (input to Arduino)
#define FpingGrndPin 26 // ping sensor ground pin (use digital pin as ground)
    
// servo
Servo steeringServo;
#define servoPin 7 // pin for servo signal
#define SERVOPOT A7
#define SERVOSCALEUS 10
#define SERVOMAXUS 400
double servoAngleDeg;

// defining global variables and constants
double dt;
unsigned long microlast;
double const velWithPWM100 = 1.40 / 2.95; // car went 1.4 m in 2.95 seconds at PWM 100;
double KI = -0.3;
double KP = -4.5;
double currAngle = 0.0;
double rb;
double desiredAngle = 0.0;
double wheelTurnBias;

void setup() {
    Serial.begin(115200);
    // servo
    steeringServo.attach(servoPin);
    setServoAngle(servoAngleDeg);

    // front ping sensor
    pinMode(FpingGrndPin,OUTPUT); digitalWrite(FpingGrndPin,LOW);
    pinMode(FpingTrigPin,OUTPUT);
    pinMode(FpingEchoPin,INPUT);

    // motor
    spinMotors();

    // IMU
    lsm.begin();
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    rb = findRB();
}

void loop() {
  // center wheels
  double frontDist = getPingDistanceCM(1);
  if (frontDist < 10.0) {
        static double servoCenter_us = 1350.0;  // default value can be changed in SETUP
        servoCenter_us = 800.0 + analogRead(SERVOPOT); 
        wheelTurnBias = constrain(servoCenter_us, servoCenter_us-SERVOMAXUS, servoCenter_us+SERVOMAXUS);
        steeringServo.writeMicroseconds(wheelTurnBias);
        currAngle = 0.0;
        Serial.println(rb);
        Serial.println(wheelTurnBias);
        return;
    }
    
    // testServo();
  // IMU setup
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  
  dt = micros() - microlast;
  currAngle += -(g.gyro.z - rb) * dt * 0.000001; // get time in seconds
  microlast = micros();
  servoAngleDeg = constrain(KP * (currAngle - desiredAngle), -25, 25);
  setServoAngle(servoAngleDeg);
//  Serial.print("\tCurrAngle: \t"); Serial.println(currAngle);
//  Serial.print("\tservoAngleDeg: \t");Serial.println(servoAngleDeg);
}

void spinMotors() {
    pinMode(motorFwdPin,OUTPUT); digitalWrite(motorFwdPin,HIGH);
    pinMode(motorRevPin,OUTPUT); digitalWrite(motorRevPin,LOW);
    pinMode(motorLPWMPin,OUTPUT); analogWrite(motorLPWMPin,motorLPWM);
    pinMode(motorRPWMPin,OUTPUT); analogWrite(motorRPWMPin,motorRPWM);  
}

void setServoAngle(double sDeg) {
    double ServoCenter_us = wheelTurnBias;
    double ServoScale_us = 10.0;    // micro-seconds per degree
    double t_us = constrain(ServoCenter_us + ServoScale_us * sDeg, ServoCenter_us-SERVOMAXUS, ServoCenter_us+SERVOMAXUS);
    steeringServo.writeMicroseconds(t_us);
}

void testServo() {
  setServoAngle(20);
  delay(1000);
  setServoAngle(10); 
  delay(1000);
  setServoAngle(0);
  delay(1000);  
  setServoAngle(-10); 
  delay(1000);
  setServoAngle(-20);
  delay(1000); 
}

// left ping sensor = 0
// front ping sensor = 1
// right ping sensor = 2
double getPingDistanceCM(int ping) {
  const long timeout_us = 3000;
  double pingDistanceCM;
  if (ping == 0) {
      digitalWrite(LpingTrigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(LpingTrigPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(LpingTrigPin, LOW);
      unsigned long echo_time;
      echo_time = pulseIn(LpingEchoPin, HIGH, timeout_us);
      if (echo_time == 0)
     {
        echo_time = timeout_us;
     }
     pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
  } else if (ping == 1) {
        digitalWrite(FpingTrigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(FpingTrigPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(FpingTrigPin, LOW);
        unsigned long echo_time;
        echo_time = pulseIn(FpingEchoPin, HIGH, timeout_us);
        if (echo_time == 0)
       {
          echo_time = timeout_us;
       }
       pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
   } else if (ping == 2) {
        digitalWrite(RpingTrigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(RpingTrigPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(RpingTrigPin, LOW);
        unsigned long echo_time;
        echo_time = pulseIn(RpingEchoPin, HIGH, timeout_us);
        if (echo_time == 0)
       {
          echo_time = timeout_us;
       }
       pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
   }
  return pingDistanceCM;
}

double findRB() {
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
  
    // find gyro bias
    double sum = 0.0;
    for (int i = 0; i < 10; i++) {
      sum += g.gyro.z;
    }
    rb = sum / 10.0;  
}
