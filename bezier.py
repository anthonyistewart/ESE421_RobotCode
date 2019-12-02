# Author: Shannon Lin Nov 2019 for ESE 421

# run as "python3 bezier.py"
# images need to be saved in same directory as code
import cv2
import numpy as np
import math
import imutils
import matplotlib.pyplot as plt

# can play around with this number to get different curvatures
bezControlThresh = 50

# calculate Bezier control points from given starting (P0) and end (P3) points
def calcControlPoints(P0, P3):
    xDiff = P3[0] - P0[0]
    x1 = int(P0[0] + xDiff / 3.0)
    x2 = int(P0[0] + (2.0*xDiff) / 3.0)

    yHalfway = P0[1] + (P3[1] - P0[1]) / 2.0
    y1 = int(yHalfway + bezControlThresh)
    y2 = int(yHalfway - bezControlThresh)
    return (x1, y1), (x2, y2)

num_total_files = 16
file_counter = 0
all_expected_x = []
all_expected_y = []
all_actual_x = []
all_actual_y = []
all_files = ['X1small_Y0bigL_east.jpg', 'X2small_Y0bigL_east.jpg', 'X2small_Y0bigR_west.jpg', 'X3big_Y1smallR_north.jpg', 'X3big_Y2smallL_south.jpg', 
    'X3big_Y3smallL_south.jpg', 'X7big_Y0smallR_south.jpg', 'X5big_Y0smallR_south.jpg', 'X5big_Y1smallL_south.jpg', 'X5big_Y1smallR_south.jpg', 
    'X5big_Y2smallL_south.jpg', 'X5big_Y2smallR_south.jpg', 'X5big_Y4smallR_south.jpg', 'X5big_Y5smallR_south.jpg', 'X5big_Y6smallL_south.jpg',
    'X5small_Y1bigR_west.jpg', 'X7small_Y1bigR_west.jpg']

# function locates cone coordinates, computes bezier curve, and displays associated image processing steps
# @inputs
# filename: name of file containing cone
# @returns computed x, y cone coordinates
def bezier(filename):
    # parse filename to find size of cone
    size = filename.split('_')
    # determine minimum threshold of contour shape to be used later based on size of cone
    if "small" in size[1]:
        MIN_THRESH = 10
    else:
        MIN_THRESH = 100

    img = cv2.imread(filename)

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

    # find contours
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if (len(contours) == 1):
        # outline all contours found if area of contour > 100
        for c in contours:
            base = tuple(c[c[:, :, 1].argmax()][0])
            if cv2.contourArea(c) > MIN_THRESH:
                # draw green contour around cone
                cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
                # draw red dot at base of cone
                cv2.circle(img, base, 5, (0, 0, 255), -1)
                cv2.imshow("Cone", img)
                cv2.waitKey(0)

        # calculate control points based on P0 (starting pt, bottom left of image) and P3 (end pt, coord of cone)
        P0 = (1, 270)
        P3 = base
        P1, P2 = calcControlPoints(P0, P3)

        # array of all four control pts x and y
        x = [P0[0], P1[0], P2[0], P3[0]]
        y = [P0[1], P1[1], P2[1], P3[1]]

        # defining num of timesteps
        step = 0.005
        steps = np.arange(0, 1, step)
        num_datapoints = int(1/step)

        bezierCurve = dict()
        bezierCurve['x'] = np.array(np.arange(num_datapoints), dtype='float32')
        bezierCurve['y'] = np.array(np.arange(num_datapoints), dtype='float32')

        # compute Bezier curve from 4 control pts
        for index in range(num_datapoints):
            # equations as given on Canvas by Bruce
            p = steps[index]
            xp = int((pow(1-p, 3)*x[0]) + (3*p*pow(1-p, 2)*x[1]) + (3*pow(p, 2)*(1-p)*x[2]) + (pow(p, 3)*x[3]))
            yp = int((pow(1-p, 3)*y[0]) + (3*p*pow(1-p, 2)*y[1]) + (3*pow(p, 2)*(1-p)*y[2]) + (pow(p, 3)*y[3]))
            bezierCurve['x'][index] = xp
            bezierCurve['y'][index] = yp
            cv2.circle(img, (xp, yp), 5, (255, 0, 0), -1)

        # show original image w Bezier curve
        cv2.imshow("bezier", img)
        cv2.waitKey(0)

        return base

# iterate over all images specified
while file_counter != num_total_files:
    # locate cone coordinates and compute bezier curve
    bezier(all_files[file_counter])
    # increment file count 
    file_counter = file_counter + 1