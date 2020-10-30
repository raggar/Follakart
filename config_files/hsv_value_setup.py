"""
Tested Values: 
- Hue_min: 007
- Hue_max: 179
- Sat_min: 000
- Sat_max: 255
- Val_min: 000
- Val_max: 255
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt

# takes in current position of trackbar (called everytime trackbar value changes)
def placeholder_method(current_position):
    print(current_position)
    pass


# Creates a window that modifies the HSV values we are looking for.
cv2.namedWindow("TrackBar")
cv2.resizeWindow("TrackBar", 1000, 2000)

# Initializing Trackbars
# cv2.createTrackbar(trackbar_name, window_name, initial_value, final_value, callback_function)

# Getting the hue (H) of the colour
cv2.createTrackbar("Hue Min", "TrackBar", 7, 255, placeholder_method)
cv2.createTrackbar("Hue Max", "TrackBar", 150, 255, placeholder_method)

# Getting the saturation (S) of the colour
cv2.createTrackbar("Sat Min", "TrackBar", 0, 255, placeholder_method)
cv2.createTrackbar("Sat Max", "TrackBar", 255, 255, placeholder_method)

# Getting the value (V) of the colour
cv2.createTrackbar("Val Min", "TrackBar", 0, 255, placeholder_method)
cv2.createTrackbar("Val Max", "TrackBar", 255, 255, placeholder_method)

image_path = 'ball_images/ball_1.jpg'

while True:
    image = cv2.imread(image_path)
    # Gets image from directory

    # Converts the image from blue green red (BGR) to HSV
    HSV_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Get values from the track bars
    hue_min = cv2.getTrackbarPos("Hue Min", "TrackBar")
    hue_max = cv2.getTrackbarPos("Hue Max", "TrackBar")
    sat_min = cv2.getTrackbarPos("Sat Min", "TrackBar")
    sat_max = cv2.getTrackbarPos("Sat Max", "TrackBar")
    val_min = cv2.getTrackbarPos("Val Min", "TrackBar")
    val_max = cv2.getTrackbarPos("Val Max", "TrackBar")

    # Arrays store the minimum and maximum values for each HSV value
    minimum_values = np.array([hue_min, sat_min, val_min])
    maximum_values = np.array([hue_max, sat_max, val_max])

    # Creates an image that displays the HSV colour indicated by the track bars
    HSV_image_specific = cv2.inRange(HSV_image, minimum_values, maximum_values)

    # Resizes images
    image = cv2.resize(image, (1000, 600))
    HSV_image_specific = cv2.resize(HSV_image_specific, (1000, 600))

    # Creating Mask
    mask = cv2.bitwise_not(HSV_image_specific)

    result = cv2.bitwise_and(image, image, mask=mask)

    # Display images
    titles = ["Image", "Mask", "Result"]
    images = [image, mask, result]
    # cv2.imshow("result", result)

    for i in range(len(images)):
        converted_image = cv2.cvtColor(images[i], cv2.COLOR_BGR2RGB)
        plt.subplot(1, 3, i + 1), plt.imshow(converted_image, "gray")
        plt.title(titles[i])
        plt.xticks([]), plt.yticks([])

    plt.show()

    if cv2.waitKey(1) & 0xFF == ord('q'):  # If 'q' is pressed, code ends
        break
