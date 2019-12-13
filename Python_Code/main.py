import math
import time
import picamera
from picamera.array import PiRGBArray
import numpy as np
from time import sleep
from Arduino_Communicator import ArduinoCommunicator
from ConeDetection import ConeDetection

coneX = 50
coneY = 50

if __name__ == "__main__":
    arduinoCom = ArduinoCommunicator(0x8)
    coneDetector = ConeDetection()

    image_size = (int(960 / 2), int(544 / 2))  # (16*photoHeight/9, photoHeight)
    camera = picamera.PiCamera()

    camera.resolution = image_size
    camera.framerate = 5
    camera.vflip = False
    camera.hflip = False
    rawCapture = PiRGBArray(camera, size=image_size)
    time.sleep(0.1)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array
        coords = coneDetector.get_cone(img)
        if coords:
            print(coords)
            """
            arduinoCom.write_float_to_register(coords[0], 1)
            arduinoCom.write_float_to_register(coords[1], 2)
            arduinoCom.trigger_method("SENT_CAMERA_DATA")
            """

