import cv2
import numpy as np

"""
Next steps:
- Find out what the x-coordinate is for the center of the image.
- Right now, the program is only detecting the ball every few frames, 
  increasing the reliability of the detection algorithm is important.
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

    image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Converts the image from blue green red (BGR) to HSV
    image_HSV_specific = cv2.inRange(image_HSV, minimum_values,
                                     maximum_values)  # Converts HSV image so only the red ball is displayed
    blur_image = cv2.blur(image_HSV_specific, (8, 8))
    contours, hierarchy = cv2.findContours(blur_image, cv2.RETR_LIST,
                                           cv2.CHAIN_APPROX_SIMPLE)  # Finds contours in image (which can be used to identify shapes)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)

    # Displays image that the computer processes
    # cv2.imshow("Processed_Image", blur_image)

    for contour in contours:
        shape_area = cv2.contourArea(contour)
        if shape_area > 2000:  # If shape area is larger than 500 (makes sure random detectoins are not processed)
            perimeter = cv2.arcLength(contour, True)
            verticies = cv2.approxPolyDP(contour, 0.03 * perimeter, True)  # Adjust values as needed

            if len(verticies) > 6:  # Circles will still have verticies. If we can find a good balance then we are solid
                x, y, width, height = cv2.boundingRect(verticies)  # Gets parameters for a rectangle around object
                object_coordinates_in_frame = [x, y]
                object_rectangle_width_height = [width, height]

                # Determines if the ball is to the left or right of the car
                if x+width/2+50 < video_frame_width: # Left
                    # cv2.putText(image, "Left", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA)
                    object_position_relative = 0
                elif x+width/2-250 > video_frame_height: # Right
                    # cv2.putText(image, "Right", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA )
                    object_position_relative = 1
                else: # Center (Dead ahead)
                    # cv2.putText(image, "Center", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA)
                    object_position_relative = 2

                cv2.rectangle(image, (x, y), (x + width, y + height), (0, 255, 0), 3)  # Draws rectangle on image

                return object_position_relative, shape_area, object_coordinates_in_frame, object_rectangle_width_height  # Exits for loop (and function) after the ball is found to reduce execution time

    return -1, -1, [-1, -1], [-1, -1] # If object not found

### METHOD BLOCK BLOCK ENDS ###



### MAIN CODE BLOCK BEGINS ###

# Video frame dimensions (480p) (funnily enough, these are also the center coordinates of the frame...)
video_frame_width = 640
video_frame_height = 480
video_brightness_adjustment = 7

video_capture = cv2.VideoCapture(0) # Captures video from computer webcam (May need to configure computer settings for this to work)
video_capture.set(3, video_frame_width)
video_capture.set(4, video_frame_height)
video_capture.set(10, video_brightness_adjustment)
# https://stackoverflow.com/questions/11420748/setting-camera-parameters-in-opencv-python --> good source to see what each .set() parameter does.

# Stores information related to the position of the object in the image
object_position_relative = 0 # 0 means left of center, 1 means right, 2 means center, -1 means not found
object_contour_area = 0 # Gives indication of how close the object is to the car
object_coordinates_in_frame = [0,0] # Coordinates of object in the frame
object_rectangle_width_height = [0, 0] # Width and height of bounding rectangle

while True:
    success, image = video_capture.read()

    object_position_relative, object_contour_area, object_coordinates_in_frame, object_rectangle_width_height = detect_object() # Calls method to process image and identify object

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

    if cv2.waitKey(1) & 0xFF == ord('q'): # If 'q' is pressed, code ends
        break

    cv2.waitKey(10)


print("Terminated")

### MAIN CODE BLOCK ENDS ###

