# Credit: Adrian Rosebrock
# https://www.pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/

# import the necessary packages
from picamera.array import PiRGBArray  # Generates a 3D RGB array
from picamera import PiCamera  # Provides a Python interface for the RPi Camera Module
import time  # Provides time-related functions
import cv2  # OpenCV library
import numpy as np
import RPi.GPIO as GPIO
import math
import pd


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
                if x + width / 2 + 50 < video_frame_width:  # Left
                    # cv2.putText(image, "Left", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA)
                    object_position_relative = 0
                elif x + width / 2 - 250 > video_frame_height:  # Right
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


def calcDistance(object_dimensions):
    percievedWidth = object_dimensions[0]
    focalLen = 612.8
    return (6 * focalLen) / percievedWidth


def calcAngle(obj_coordinates, distance):
    pixelDistanceFromCenter = abs(320 - obj_coordinates[0])
    angle = math.degrees(math.atan((pixelDistanceFromCenter * 0.015) / distance))
    return angle


### METHOD BLOCK BLOCK ENDS ###

# Initialize the camera
camera = PiCamera()

# Set the camera resolution
camera.resolution = (640, 480)

# Set the number of frames per second
camera.framerate = 32
video_frame_width = 640
video_frame_height = 480
video_brightness_adjustment = 7

# Information about Object
object_position_relative = 0  # 0 means left of center, 1 means right, 2 means center, -1 means not found
object_contour_area = 0  # Gives indication of how close the object is to the car
object_coordinates_in_frame = [0, 0]  # Coordinates of object in the frame
object_rectangle_width_height = [0, 0]  # Width and height of bounding rectangle

# Positioning Characteristics
desired_angle = 0
current_angle = 0
desired_distance = 0
current_distance = 0

# Generates a 3D RGB array and stores it in rawCapture
raw_capture = PiRGBArray(camera, size=(640, 480))

# Wait a certain number of seconds to allow the camera time to warmup
time.sleep(0.1)


# Configures GPIO
# We will need to test values to find the optimum ones
angle_pid = pd.PD(5, 2, 65, 90)  # Constructs PD class for when the robot needs to rotate
forward_pid = pd.PD(12, 5, 65, 90)  # Constructs PD class for when the robot needs to move forwards and backwards

desired_angle = 0  # We want the ball to be at 0 degrees relative to the centre
desired_distance = 10;  # DECIDE THIS

# Define ports for motor 1 (Left)
motor_1_forward = 24
motor_1_backward = 23
motor_1_en = 25

# Define ports for motor 2 (Right)
motor_2_forward = 27
motor_2_backward = 22
motor_2_en = 17

# Initialize GPIO

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_1_forward, GPIO.OUT)
GPIO.setup(motor_1_backward, GPIO.OUT)
GPIO.setup(motor_1_en, GPIO.OUT)
GPIO.output(motor_1_forward, GPIO.LOW)
GPIO.output(motor_1_backward, GPIO.LOW)
GPIO.setup(motor_2_forward, GPIO.OUT)
GPIO.setup(motor_2_backward, GPIO.OUT)
GPIO.setup(motor_2_en, GPIO.OUT)
GPIO.output(motor_2_forward, GPIO.LOW)
GPIO.output(motor_2_backward, GPIO.LOW)
pwm_motor_1 = GPIO.PWM(motor_1_en, 1000)
pwm_motor_2 = GPIO.PWM(motor_2_en, 1000)

# Good PWM values to move straight
pwm_motor_1.start(75)
pwm_motor_2.start(70)

# Capture frames continuously from the camera
for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):

    # Grab the raw NumPy array representing the image
    image = frame.array

    # Display the frame using OpenCV
    object_position_relative, object_contour_area, object_coordinates_in_frame, object_rectangle_width_height = detect_object()  # Calls method to process image and identify object

    # print(obje/ct_rectangle_width_height)

    cv2.imshow("Webcam_Input", image)

    current_distance = calcDistance(object_rectangle_width_height)
    current_angle = calcAngle(object_coordinates_in_frame, current_distance)

    if object_position_relative == 0 or object_position_relative == 1:
        motor_pwm = angle_pid.get_output(current_angle, desired_angle)
    elif object_position_relative == 2 or object_position_relative == -1:
        motor_pwm = forward_pid.get_output(current_distance, desired_distance)

    print(current_distance, " ", current_angle, " ", motor_pwm, " ", object_position_relative)

    if object_position_relative == 0:  # Left
        pwm_motor_2.ChangeDutyCycle(motor_pwm)
        time.sleep(0.05)
        GPIO.output(motor_1_forward, GPIO.LOW)
        GPIO.output(motor_1_backward, GPIO.LOW)
        GPIO.output(motor_2_forward, GPIO.HIGH)
        GPIO.output(motor_2_backward, GPIO.LOW)
        time.sleep(0.15)
    elif object_position_relative == 1:  # Right
        pwm_motor_1.ChangeDutyCycle(motor_pwm)
        time.sleep(0.05)
        GPIO.output(motor_1_forward, GPIO.HIGH)
        GPIO.output(motor_1_backward, GPIO.LOW)
        GPIO.output(motor_2_forward, GPIO.LOW)
        GPIO.output(motor_2_backward, GPIO.LOW)
        time.sleep(0.15)
    elif object_position_relative == 2:  # Centre
        if current_distance > desired_distance:  # If car is farther than desired target
            pwm_motor_1.ChangeDutyCycle(motor_pwm)
            pwm_motor_2.ChangeDutyCycle(motor_pwm)
            time.sleep(0.05)
            GPIO.output(motor_1_forward, GPIO.HIGH)
            GPIO.output(motor_1_backward, GPIO.LOW)
            GPIO.output(motor_2_forward, GPIO.HIGH)
            GPIO.output(motor_2_backward, GPIO.LOW)
        elif current_distance < desired_distance:  # If car is closer than desired target
            pwm_motor_1.ChangeDutyCycle(motor_pwm)
            pwm_motor_2.ChangeDutyCycle(motor_pwm)
            time.sleep(0.05)
            GPIO.output(motor_1_forward, GPIO.LOW)
            GPIO.output(motor_1_backward, GPIO.HIGH)
            GPIO.output(motor_2_forward, GPIO.LOW)
            GPIO.output(motor_2_backward, GPIO.HIGH)
        time.sleep(0.2)
    elif object_position_relative == -1:  # Ball not found
        pwm_motor_2.ChangeDutyCycle(80)
        time.sleep(0.05)
        GPIO.output(motor_1_forward, GPIO.LOW)
        GPIO.output(motor_1_backward, GPIO.LOW)
        GPIO.output(motor_2_forward, GPIO.HIGH)
        GPIO.output(motor_2_backward, GPIO.LOW)
        time.sleep(0.4)

    GPIO.output(motor_1_forward, GPIO.LOW)
    GPIO.output(motor_1_backward, GPIO.LOW)
    GPIO.output(motor_2_forward, GPIO.LOW)
    GPIO.output(motor_2_backward, GPIO.LOW)

    # Clear the stream in preparation for the next frame
    raw_capture.truncate(0)

    time.sleep(0.1)

print("Done")

pwm_motor_1.stop()
pwm_motor_2.stop()
GPIO.cleanup()  # Sets all pins to input to prevent short circutings

