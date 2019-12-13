import cv2
import math
import time
import consts
import imutils
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from random import random


class ConeDetection:

    def get_coordinates(self, pos):
        yi_meters = pos[0] * 1.12 * pow(10, -6)
        zi_meters = pos[1] * 1.12 * pow(10, -6)
        c_x_p = (consts.h * consts.f) / zi_meters
        c_y_p = (yi_meters * c_x_p) / consts.f
        c_y_p_2 = (consts.h * yi_meters) / zi_meters
        return c_x_p, c_y_p, c_y_p_2

    def find_cone(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        h = cv2.equalizeHist(h)
        s = cv2.equalizeHist(s)
        v = cv2.equalizeHist(v)
        thresh_h = cv2.bitwise_not(cv2.inRange(h, consts.lower_b[0], consts.upper_b[0]))
        thresh_s = cv2.inRange(s, consts.lower_b[1], consts.upper_b[1])
        thresh_v = cv2.inRange(v, consts.lower_b[2], consts.upper_b[2])
        mask = thresh_h & thresh_v & thresh_s
        return mask

    def find_cone2(self, img):
        b, g, r = cv2.split(img)
        red = cv2.equalizeHist(r)
        green = cv2.equalizeHist(g)
        blue = cv2.equalizeHist(b)
        equalized = cv2.merge((blue, green, red))

        # find mask that has colors within lowerBound and upperBound
        mask = cv2.inRange(equalized, lb, ub)

        # find contours in mask image
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)



    def find_base(self, mask, offset=None):
        corrected_pos, base_pos = None, None
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        if len(contours) > 0:
            for c in contours:
                if cv2.contourArea(c) > consts.contour_threshold:
                    base_pos = tuple(c[c[:, :, 1].argmax()][0])
                    if offset:
                        corrected_pos = (base_pos[0] - int((consts.img_width / 2)) + offset[0]), (
                                    base_pos[1] - int((consts.img_height / 2)) + offset[1])
                    else:
                        corrected_pos = (base_pos[0] - int((consts.img_width / 2)) + consts.offset[0]), (
                                    base_pos[1] - int((consts.img_height / 2)) + consts.offset[1])

        return corrected_pos, base_pos, contours

    def get_cone(self, img):
        # Find the cone and display generated mask
        mask = self.find_cone(img)

        # Find the base of the cone and save the position
        base_img = img.copy()
        corrected_coords, img_coords, contours = self.find_base(mask)
        if not img_coords:
            return None
        return img_coords


