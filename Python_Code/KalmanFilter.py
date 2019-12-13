import numpy as np
import math


class KalmanFilter:

    MIN_DIST = 10  # Robot must be at least 10cm away from cone to move onto the next
    DEFAULT_PWM = 100  # default PWM
    K_PSI = 1.5  # Heading Feeback

    def __init__(self):
        self.I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # identity matrix
        self.P = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])  # error Covariance Matrix
        self.P_prime = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])  # error Covariance Matrix predicted by Kalman Filter
        self.P_last = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])  # previous Error Covariance Matrix predicted by Kalman Filter
        self.x_hat = np.array([0, 0, 0]) # best estimate of the X vector given by the Kalman Filter
        self.x_hat_prime = np.array([0, 0, 0]) # X vector predicted by thes Kalman Filter
        self.x_hat_last = np.array([0, 0, 0]) # previous X vector predicted by the Kalman Filter

        self.Q = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])  # actual mvt variance (car mechanics)
        self.R = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])  # measurement variance
        self.K_k = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])  # Kalman gain

        self.A_k = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
        self.L = 0

    # helper functions to set Q, R, L attributes of KalmanFilter object
    def setQ(self, Q):
        self.Q = Q

    def setR(self, R):
        self.R = R

    def setL(self, L):
        self.L = L

    # function performs Kalman filter with no camera/measurement input
    # @inputs
    # u: <2,1> matrix containing vel in cm/s, gyro.z
    # dt_kalman: change in time since last Kalman calculation
    # @returns <3,1> matrix with predicted x, y, psi
    def predictionNoCamera(self, u, dt):
        # Project the state ahead
        self.x_hat_prime[2]= self.x_hat_last[2] + dt*(u[0]/self.L)*u[1];  # Calculate psi
        self.x_hat_prime[0] = self.x_hat_last[0] + dt*u[0]*math.cos(self.x_hat_prime[2]); # Calculate X
        self.x_hat_prime[1] = self.x_hat_last[1] + dt*u[0]*math.sin(self.x_hat_prime[2]); # Calculate Y
        

        # Project the covariance error ahead
        self.P_prime = (self.A_k * self.P_last * np.transpose(self.A_k) +self.Q)

        # Store values for next prediction
        self.x_hat_last = self.x_hat_prime
        self.P_last = self.P_prime
    
        return self.x_hat_prime

    # function calculates distance from current position to cone
    # @inputs
    # x_k: <3,1> matrix x, y, psi
    # cone: <2,1> matrix x, y
    # @returns distance to cone
    def distToCone(self,x_k, cone):
        return math.sqrt(math.pow(x_k[0] - cone[0], 2) + math.pow(x_k[1] - cone[1], 2))

    # function calculates angle from current position to cone
    # @inputs
    # x_k: <3,1> matrix x, y, psi
    # cone: <2,1> matrix x, y
    # @returns angle to cone
    def angleToCone(self,x_l, cone):
        return 90 - (180/math.pi * np.arctan(x_k[1] - cone[1]) / x_k[0] - cone[0])

    # function computes PWM and servoAngle to send back to Arduino
    # @inputs
    # x_k: <3,1> matrix x, y, psi
    # x: x coordinate of detected cone
    # y: y coordinate of detected cone
    # @returns <2,1> matrix PWM and servoAngle
    def sendToArduino(self,x_k, x, y):
        pwm_angle = np.array([Kalman.DEFAULT_PWM, 0])
        cone = np.array([x, y])
        if (self.distToCone(x_k, cone) <= Kalman.MIN_DIST):
            pwm_angle[0] = 0
    
        # desiredHeading = angleToCone(x_k, cone)
        desiredHeading = 90
        error = desiredHeading - x_k[2]
        pwm_angle[1] = Kalman.K_PSI * error
        
        return pwm_angle

