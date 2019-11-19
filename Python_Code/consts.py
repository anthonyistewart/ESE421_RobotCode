import math

# Set this to true to display all images one by one
# Otherwise it will run through and plot the data instead
DISPLAY_IMG = False

# Height and Width of image
img_height = 272
img_width = 480

# Pinhole height in meters
h = 0.135

# Focal length in meters
f = 3.04 * pow(10, -3)

#Center Point Calibration
offset = [8, -6]

# Robot Constraints
delta_max = 20
L = 0.18 # meters
k_max = math.tan(delta_max)/L

# HSV bounds
lower_b = [12, 220, 100]
upper_b = [240, 255, 255]

# Contour Threshold
contour_threshold = 150

# Bezier Curve Step Size
step = 0.0001