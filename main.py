import numpy as np
import math
import Arduino_Communicator

arduinoCom = ArduinoCommunicator(0x8)
r_imu = get_arduino_data_from_register(arduinoCom, 1)
velocity = get_arduino_data_from_register(arduinoCom, 2)
dt_kalman = get_arduino_data_from_register(arduinoCom, 3)
u = np.array([r_imu, velocity])
x_k = predictionNoCamera(u, dt_kalman)
arduinoCom.write_float_to_register(x_k[0], 4) # register 4 stores PWM
arduinoCom.write_float_to_register(x_k[1], 5) # register 5 stores servoAngleDeg
arduinoCom.trigger_method("SENT")
coneX = 50
coneY = 50
sendToArduino(x_k, coneX, coneY)
