import cv2
import numpy as np

"""
Next steps:
- Find out what the x-coordinate is for the center of the image.
- Right now, the program is only detecting the ball every few frames, 
  increasing the reliability of the detection algorithm is important.
- Double check code for determining left and right relative positions (and provide actual values)
"""

### GLOBAL VARIABLES BEGINS ###

### GLOBAL VARIABLES ENDS ###

### METHOD BLOCK BEGINS ###


def detect_object():
    """
    Possible optimization steps for the future:
    - Change blur values
    - Change minimum verticies count
    - Change minimum shape area count
    """

    # Arrays store the minimum and maximum values for each HSV value. Refer to "hsv_value_setup.py"
    minimum_values = np.array([7, 0, 0])
    maximum_values = np.array([150, 255, 255])

    # Converts the image from blue green red (BGR) to type HSV
    image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    image_HSV_specific = cv2.inRange(image_HSV, minimum_values, maximum_values)

    blur_image = cv2.blur(image_HSV_specific, (8, 8))  # kernal of 8x8 used

    # Finds contours in image (which can be used to identify shapes)
    contours, _ = cv2.findContours(
        blur_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
    # each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.

    for contour in contours:
        shape_area = cv2.contourArea(contour)

        # If shape area is larger than 500 (makes sure random detectoins are not processed)
        if shape_area > 2000:
            perimeter = cv2.arcLength(contour, True)
            # accuracy parameter - maximum distance from contour to approximated contour
            episilon = 0.03 * perimeter
            verticies = cv2.approxPolyDP(
                contour, episilon, True)  # Adjust values as needed

            # Circles will still have verticies. If we can find a good balance then we are solid
            if len(verticies) > 6:
                # Gets parameters for a rectangle around object
                x, y, width, height = cv2.boundingRect(verticies)
                # x,y are top left coordinates of rectangle
                object_coordinates_in_frame = [x, y]
                object_rectangle_width_height = [width, height]

                # Determines if the ball is to the left or right of the car
                if x+width/2+50 < video_frame_width:  # Left
                    # cv2.putText(image, "Left", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA)
                    object_position_relative = 0
                elif x+width/2-250 > video_frame_height:  # Right
                    # cv2.putText(image, "Right", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA )
                    object_position_relative = 1
                else:  # Center (Dead ahead)
                    # cv2.putText(image, "Center", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA)
                    object_position_relative = 2

                cv2.rectangle(image, (x, y), (x + width, y + height),
                              (0, 255, 0), 3)  # Draws rectangle on image

                # Exits for loop (and function) after the ball is found to reduce execution time
                return object_position_relative, shape_area, object_coordinates_in_frame, object_rectangle_width_height

    return -1, -1, [-1, -1], [-1, -1]  # If object not found


### MAIN CODE BLOCK BEGINS ###
# Video frame dimensions (480p) (funnily enough, these are also the center coordinates of the frame...)
video_frame_width = 640
video_frame_height = 480
video_brightness_adjustment = 7

# Captures video from computer webcam (May need to configure computer settings for this to work)
video_capture = cv2.VideoCapture(0)
video_capture.set(3, video_frame_width)
video_capture.set(4, video_frame_height)
video_capture.set(10, video_brightness_adjustment)
# https://stackoverflow.com/questions/11420748/setting-camera-parameters-in-opencv-python --> good source to see what each .set() parameter does.

# 0 means left of center, 1 means right, 2 means center, -1 means not found
object_position_relative = 0
# Gives indication of how close the object is to the car (higher value = closer)
object_contour_area = 0
object_coordinates_in_frame = [0, 0]
object_rectangle_width_height = [0, 0]

while True:
    _, image = video_capture.read()

    # Calls method to process image and identify object
    object_position_relative, object_contour_area, object_coordinates_in_frame, object_rectangle_width_height = detect_object()

    cv2.imshow("Webcam_Input", image)

    # print(object_position_relative, " ", object_contour_area, " ", object_coordinates_in_frame, " ", object_rectangle_width_height)

    """
    Steps to add next:
    - Determine the motor power output required to get the object to the center of the frame (or for it to become larger)
    - Power motors using PWM, and use a PID to constantly modify the power output
    """

    # if object_position_relative == 0: # Left
    #
    # elif object_position_relative == 1: # Right
    #
    # else: # center

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cv2.waitKey(10)

video_capture.release()
print("Terminated")

### MAIN CODE BLOCK ENDS ###
