import cv2
import math
import time
import consts
import imutils
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from random import random


# Calculates the control points of the Bezier Curve
def find_control_points(P1, P4):
    x_diff = P4[0] - P1[0]
    x1 = P1[0] + x_diff / 3.0
    x2 = P1[0] + (2.0 * x_diff) / 3.0

    """
    # Lines along control points
    m1 = 1
    m2 = 1

    for i in range(0, 100):
        y1 = m1*i
        y2 = m2*i
        k1 = np.cross(tuple(numpy.subtract(P2, P1)), tuple(numpy.subtract(P3, P2)))
    """
    y1 = P1[1]
    y2 = P4[1]
    return P1, (x1, y1), (x2, y2), P4


def find_bezier_curve(points):
    steps = np.arange(0, 1, consts.step)
    num_datapoints = int(1 / consts.step)

    camera_plane = dict()
    camera_plane['x'] = np.array(np.arange(num_datapoints), dtype='float32')
    camera_plane['y'] = np.array(np.arange(num_datapoints), dtype='float32')

    image_plane = dict()
    image_plane['x'] = np.array(np.arange(num_datapoints), dtype='float32')
    image_plane['y'] = np.array(np.arange(num_datapoints), dtype='float32')

    for index in range(num_datapoints):
        p = steps[index]
        xp = (pow(1 - p, 3) * points[0][0]) + (3 * p * pow(1 - p, 2) * points[1][0]) + (
                3 * pow(p, 2) * (1 - p) * points[2][0]) + (pow(p, 3) * points[3][0])
        yp = (pow(1 - p, 3) * points[0][1]) + (3 * p * pow(1 - p, 2) * points[1][1]) + (
                3 * pow(p, 2) * (1 - p) * points[2][1]) + (pow(p, 3) * points[3][1])
        image_plane['x'][index] = xp
        image_plane['y'][index] = yp

        camera_plane['x'][index], camera_plane['y'][index], _ = get_coordinates((xp, yp))

    return image_plane, camera_plane


def get_coordinates(pos):
    yi_meters = pos[0] * 1.12 * pow(10, -6)
    zi_meters = pos[1] * 1.12 * pow(10, -6)
    c_x_p = (consts.h * consts.f) / zi_meters
    c_y_p = (yi_meters * c_x_p) / consts.f
    c_y_p_2 = (consts.h * yi_meters) / zi_meters
    return c_x_p, c_y_p, c_y_p_2


# Returns a cv2 image and actual camera position
def _load_image(path):
    filepath = Path(path)
    info = filepath.stem.split('_')
    x = int(info[0][1:2])
    y = int(info[1][1:2])

    # Calculate actual position
    if 'big' in info[0]:
        x *= 0.92
    else:
        x *= 0.405
    if 'big' in info[1]:
        y *= 0.92
    else:
        y *= 0.405

    # Correct Position sign
    if 'L' in info[1]:
        y *= -1

    actual_position = (x, y)
    image = cv2.imread(str(filepath))
    return image, actual_position


# Loads all of the images in a given directory and returns a list containing tuples
# with: (filename, cv2 image, actual position)
def load_all_images(dir):
    images = []
    filepaths = list(Path(dir).rglob('*.jpg'))
    for f in filepaths:
        eq_img, pos = _load_image(f)
        images.append((f.stem, eq_img, pos))
    return images


def find_cone(image):
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


def find_base(mask, offset=None):
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


def plot_data(actuals, predictions):
    # Plot the first graph
    plt.figure()
    plt.scatter(y=[i[0] * 100 for i in actuals], x=[i[1] * 100 for i in actuals], label='Actual', s=80,
                marker='x', color='k')
    plt.scatter(y=[i[0] * 100 for i in predictions], x=[i[1] * 100 for i in predictions], label='Predicted', s=80,
                facecolors='none', marker='o', color='r')

    plt.title('Cone Position')
    plt.xlabel('Y Position (cm)')
    plt.ylabel('X Position (cm)')
    plt.legend()
    plt.xlim(-300, 300)
    plt.ylim(-300, 300)
    plt.show()

    # Plot the second graph
    plt.figure()

    plt.scatter(x=[i[0] * 100 for i in actuals], y=[i[0] * 100 for i in predictions], label='cXp', s=80,
                facecolors='none', marker='s', color='b')
    plt.scatter(x=[i[1] * 100 for i in actuals], y=[i[1] * 100 for i in predictions], label='cYp from iYp', s=80,
                facecolors='none', marker='o', color='r')
    plt.scatter(x=[i[1] * 100 for i in actuals], y=[i[2] * 100 for i in predictions], label='cYp from h', s=80,
                facecolors='none', marker='^', color='g')

    plt.title('Cone Position')
    plt.xlabel('Actual Value (cm)')
    plt.ylabel('Predicted Value (cm)')
    plt.legend()
    plt.xlim(-300, 300)
    plt.ylim(-300, 300)
    x = np.linspace(-300, 300)
    plt.plot(x, x, '--', linewidth=2, color='k')
    plt.show()


if __name__ == "__main__":
    failed = 0
    images = load_all_images('test_images')
    actual = []
    prediction = []
    for img in images:
        if consts.DISPLAY_IMG:
            # Display original image
            cv2.imshow(img[0], img[1])

            # Find the cone and display generated mask
            mask = find_cone(img[1])
            cv2.imshow(img[0] + " - Cone Detection", mask)

            # Find the base of the cone and save the position
            base_img = img[1].copy()
            corrected_coords, img_coords, contours = find_base(mask)
            if not corrected_coords:
                break

            # Display the contours found
            contour_img = img[1].copy()
            cv2.drawContours(contour_img, contours, -1, (0, 0, 255), -1)
            cv2.imshow("Contours", contour_img)

            # Display cone with red dot at base
            cv2.circle(base_img, img_coords, 5, (0, 0, 255), -1)
            cv2.imshow(img[0] + " - Cone Location", base_img)

            # Get World Coordinates
            corrected_predicted_coords = get_coordinates(corrected_coords)
            predicted_coords = get_coordinates(img_coords)

            print("Actual Position: {}".format(img[2]))
            print("Image Position: {}".format(img_coords))
            print("Corrected Image Position: {}".format(corrected_coords))
            print("Predicted Position: {}".format(predicted_coords))
            print("Predicted Position using Corrected Position: {}".format(corrected_predicted_coords))

            # Find the Bezier Curve in the image and camera plane
            P1, P2, P3, P4 = find_control_points((consts.img_width / 2, consts.img_height), img_coords)
            image_plane_path, camera_plane_path = find_bezier_curve([P1, P2, P3, P4])

            plt.plot(camera_plane_path['x'] * 100, camera_plane_path['y'] * 100)
            plt.title('Bezier Curve: {}'.format(img[0]))
            plt.xlabel('cXp (cm)')
            plt.ylabel('cYp (cm)')
            plt.show()

            path_img = img[1].copy()
            color = (0, 0, 255)
            for i in range(1, int(1 / consts.step)):
                coord = (image_plane_path['x'][i], image_plane_path['y'][i])
                cv2.circle(path_img, coord, 1, color, 1)
            cv2.imshow(img[0] + " - Bezier Curve", path_img)

            while True:
                if cv2.waitKey(1) == 27:
                    break
            time.sleep(0.01)
            if cv2.waitKey(1) == 27:
                break
            cv2.destroyAllWindows()
        else:
            # Find the cone and get mask
            mask = find_cone(img[1])

            # Find the base of the cone and save the position
            corrected_coords, img_coords, _ = find_base(mask, None)
            if not corrected_coords:
                failed += 1
                break

            # Get Camera Coordinates
            corrected_predicted_coords = get_coordinates(corrected_coords)
            predicted_coords = get_coordinates(img_coords)

        # Add the actual coordinates and predicted coordinates to a list for plotting
        actual.append(img[2])
        # print(corrected_predicted_coords)
        prediction.append(corrected_predicted_coords)

    print("Failed to find {} cone(s).".format(failed))
    # Plot the graph of the Actual Positions and Predicted Positions
    plot_data(actual, prediction)
