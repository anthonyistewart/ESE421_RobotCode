// Define Robot Status Flags
#define INIT 0
#define RUNNING 1
#define CALIBRATE 2

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
const int calibrationTime = 5000;  // Calibration time in milliseconds
const double Kp = 5.0;  // Proportional Feedback
const double K_psi = 1.5;  // Heading Feeback
const double w_theta = 0.5;  //Theta Filter Cutoff
const double g = 9.81; //Acceleration due to gravity
const double L = 18.0; //Length of robot in cm
const double velocity = 100; // 100cm/s
