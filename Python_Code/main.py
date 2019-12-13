import math
import numpy as np
from time import sleep
# from KalmanFilter import KalmanFilter
from Arduino_Communicator import ArduinoCommunicator
from ConeDetection import ConeDetection

coneX = 50
coneY = 50

if __name__ == "__main__":
    arduinoCom = ArduinoCommunicator(0x8)
    coneDetector = ConeDetection()

    while True:
        if coneDetector.isCone():
            c_x_p, c_y_p = coneDetector.getConePosition()
            arduinoCom.write_float_to_register(c_x_p, 1)
            arduinoCom.write_float_to_register(c_x_p, 2)
            arduinoCom.trigger_method("SENT_CAMERA_DATA")

