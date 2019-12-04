import picamera
from picamera.array import PiRGBArray
import cv2
import numpy as np;
import imutils
import re
import matplotlib.pyplot as plt
import time

camera = picamera.PiCamera()
for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):
    try:
        img = frame.array
        # split image into separate BGR channels and equalize each of them
        b, g, r = cv2.split(img)
        red = cv2.equalizeHist(r)
        green = cv2.equalizeHist(g)
        blue = cv2.equalizeHist(b)
        equalized = cv2.merge((blue, green, red))

        # find mostly red pixels, note that array is BGR
        # range based on which way it's facing
        if "north.jpg" in filename:
            lb = np.array([130, 90, 180])
            ub = np.array([165, 120, 220])
        elif ("east" in filename) | ("west" in filename):
            lb = np.array([60, 60, 250])
            ub = np.array([255, 255, 255])
        else:
            lb = np.array([50, 50, 250])
            ub = np.array([230, 230, 255])

        # find mask that has colors within lowerBound and upperBound
        mask = cv2.inRange(equalized, lb, ub)
        cv2.imshow('img', img)
        cv2.waitKey(0)
        # cv2.imshow('equalized', equalized)
        # cv2.waitKey(0)
        # cv2.imshow('mask', mask)
        # cv2.waitKey(0)

        # find contours in mask image
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours  = imutils.grab_contours(contours)
        
        # parse filename to find size of cone based on how far away it is
        size = filename.split('_')
        # determine minimum threshold of contour shape to be used later based on size of cone
        if "small" in size[1]:
            MIN_THRESH = 10
        else:
            MIN_THRESH = 150

        # outline all contours found if area of contour > MIN_THRESH
        for c in contours :
            if cv2.contourArea(c) > MIN_THRESH:
                # draw green contour around cone
                cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
                base = tuple(c[c[:, :, 1].argmax()][0])
                print("pixel(x,y): ", base)
                # draw red dot at base of cone
                cv2.circle(img, base, 5, (0, 0, 255), -1)
                cv2.imshow("Cone", img)
                cv2.waitKey(0)

        # convert pixel (x,y) coordinate to image(x,y)
        # 1 pixel = 1.12um
        i_y_p = (base[0] - 240) * 1.12 * 10**-6 # half width is 240
        i_z_p = (base[1] - 135) * 1.12 * 10**-6 # half height is 135

        # compute c_x_p, c_y_p from image(x,y)
        # focal = 400*1.12um, height= 13.4cm
        h = 13.4 * 10**-2
        f = 400 * 1.12 * 10**-6
        c_x_p = h * f / i_z_p 
        c_y_p = h * i_y_p / i_z_p
        print("(c_x_p, c_y_p): ", c_x_p, ",", c_y_p)
        all_expected_x.append(c_x_p)
        all_expected_y.append(c_y_p)

        # compute actual x, y
        # extract number of tiles in x direction and what size (big vs small)
        segments = filename.split("_")
        x_dir = re.split("([0-9])", segments[0])
        small_tile = 40.5 * 10**-2
        big_tile = 92 * 10**-2
        if "big" in x_dir[2]:
            actual_x = (float(x_dir[1]) * big_tile)
        else:
            actual_x = (float(x_dir[1]) * small_tile)

        # same logic as above
        y_dir = re.split("([0-9])", segments[1])
        if "big" in y_dir[2]:
            actual_y = (float(y_dir[1]) * big_tile) * 100 # convert to cm
        else:
            actual_y = (float(y_dir[1]) * small_tile) * 100 # convert to cm

        # send actual_x and actual_y to Arduino
        arduinoCom = ArduinoCommunicator(0x8)
        arduinoCom.write_float_to_register(actual_x, 4) # register 4 stores actual_x
        arduinoCom.write_float_to_register(actual_y, 5) # register 5 stores actual_y
        time.sleep(1)

    except:
        pass