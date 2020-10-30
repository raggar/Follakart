# import the necessary packages
from picamera.array import PiRGBArray  # Generates a 3D RGB array
from picamera import PiCamera  # Provides a Python interface for the RPi Camera Module
import time  # Provides time-related functions
import cv2  # OpenCV library
import numpy as np
import RPi.GPIO as GPIO
import math
import pd

### METHOD BLOCK BEGINS ###

def detect_object(image):

    # Arrays store the minimum and maximum values for each HSV value. Refer to "hsv_value_setup.py"
    minimum_values = np.array([7, 0, 0])
    maximum_values = np.array([150, 255, 255])

    # Converts the image from BGR to type HSV
    image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    image_HSV_specific = cv2.inRange(image_HSV, minimum_values, maximum_values)

    blur_image = cv2.blur(image_HSV_specific, (8, 8))  # kernal of 8x8 used

    # Finds contours in image (which can be used to identify shapes)
    contours, _ = cv2.findContours(blur_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

    # each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.
    for contour in contours:
        shape_area = cv2.contourArea(contour)

        if shape_area > 2400 and shape_area < 160000:
            perimeter = cv2.arcLength(contour, True)

            # accuracy parameter - maximum distance from contour to approximated contour
            episilon = 0.03 * perimeter
            verticies = cv2.approxPolyDP(contour, episilon, True)  # Adjust values as needed

            # Circles will still have verticies. If we can find a good balance then we are solid
            if len(verticies) > 6:

                # Gets parameters for a rectangle around object
                x, y, width, height = cv2.boundingRect(verticies)

                object_center_coordinates = [x + width/2, y - height/2]
                object_rectangle_width_height = [width, height]

                if object_center_coordinates[0] < (video_frame_width/2-30):  # Left
                    object_position_relative = 0
                elif object_center_coordinates[0] > (video_frame_width/2+30):  # Right
                    object_position_relative = 1
                else:
                    object_position_relative = 2

                cv2.rectangle(image, (x, y), (x + width, y + height), (0, 255, 0), 3)  # Draws rectangle on image

                # Exits for loop (and function) after the ball is found to reduce execution time
                return object_position_relative, shape_area, object_center_coordinates, object_rectangle_width_height

    return -1, -1, [-1, -1], [-1, -1] # If object not found

def calcDistance(object_dimensions):
    focalLen = 612.8
    return (6 * focalLen) / object_dimensions[0]

def calcAngle(obj_coordinates, distance):
    pixelDistanceFromCenter = abs(320 - obj_coordinates[0])
    angle = math.degrees(math.atan((pixelDistanceFromCenter * 0.015) / distance))
    return angle

### METHOD BLOCK ENDS ###



### MAIN BLOCK BEGINS ###

# Initialize the Pi camera
camera = PiCamera()

# Set the camera resolution
camera.resolution = (640, 480)

# Set the number of frames per second
camera.framerate = 32
video_frame_width = 640
video_frame_height = 480
video_brightness_adjustment = 7

# Positioning Characteristics
desired_angle = 0
current_angle = 0
desired_distance = 0
current_distance = 0

raw_capture = PiRGBArray(camera, size=(640, 480)) # Generates a 3D RGB array and stores it in rawCapture
time.sleep(0.1) # Wait a certain number of seconds to allow the camera time to warmup

angle_pd = pd.PD(7, 2, 65, 90)  # Constructs PD class for when the robot needs to rotate
forward_pd = pd.PD(12, 5, 65, 90)  # Constructs PD class for when the robot needs to move forwards and backwards

# Desired position for the ball with respect to the car
desired_angle = 0
desired_distance = 30

# Define ports for motor 1 (Left)
motor_1_forward = 24
motor_1_backward = 23
motor_1_en = 25

# Define ports for motor 2 (Right)
motor_2_forward = 22
motor_2_backward = 27
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

# Starts PWMs
pwm_motor_1.start(75)
pwm_motor_2.start(70)

# Information about Object
object_position_relative = 0  # 0 means left of center, 1 means right, 2 means center, -1 means not found
object_contour_area = 0  # Gives indication of how close the object is to the car
object_coordinates_in_frame = [0, 0]  # Coordinates of object in the frame
object_rectangle_width_height = [0, 0]  # Width and height of bounding rectangle

undetected_counter = 0

for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True): # Capture frames continuously from the camera

    image = frame.array # Grab the raw NumPy array representing the image

    object_position_relative, object_contour_area, object_center_coordinates, object_rectangle_width_height = detect_object(image)  # Calls method to process image and identify object

    cv2.imshow("Webcam_Input", image)
    cv2.waitKey(50)

    raw_capture.truncate(0) # Clear the stream in preparation for the next frame

    if object_contour_area != -1: # If object is found
        # Calculates distances
        current_distance = calcDistance(object_rectangle_width_height)
        current_angle = calcAngle(object_coordinates_in_frame, current_distance)

        # Determines required motor output
        if object_position_relative == 0 or object_position_relative == 1:
            motor_pwm = angle_pd.get_output(current_angle, desired_angle)
        elif object_position_relative == 2 or object_position_relative == -1:
            motor_pwm = forward_pd.get_output(current_distance, desired_distance)

        # print(current_distance, " ", current_angle, " ", motor_pwm, " ", object_position_relative, " ", object_center_coordinates, " ", object_rectangle_width_height)  # Debugging

        # Powers motors
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
            if current_distance > desired_distance:  # If car is farther than desired targeta
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

    else:  # Object not found
        undetected_counter = undetected_counter + 1
        if undetected_counter == 5:
            undetected_counter = 0
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

    time.sleep(0.05)

print("Done")

pwm_motor_1.stop()
pwm_motor_2.stop()
GPIO.cleanup()

### MAIN BLOCK ENDS ###

