import cv2
import numpy as np

# Video frame dimensions (480p) (funnily enough, these are also the center coordinates of the frame...)
video_frame_width = 640
video_frame_height = 480
video_brightness_adjustment = 7

# Captures video from computer webcam (May need to configure computer settings for this to work)
video_capture = cv2.VideoCapture("test.mp4")
video_capture.set(3, video_frame_width)
video_capture.set(4, video_frame_height)
video_capture.set(10, video_brightness_adjustment)
# https://stackoverflow.com/questions/11420748/setting-camera-parameters-in-opencv-python --> good source to see what each .set() parameter does.

# 0 means left of center, 1 means right, 2 means center, -1 means not found
object_contour_area = 0
object_center_coordinates = [0, 0]
object_rectangle_width_height = [0, 0]


def detect_object(success, image):

    # Arrays store the minimum and maximum values for each HSV value. Refer to "hsv_value_setup.py"
    minimum_values = np.array([7, 0, 0])
    maximum_values = np.array([150, 255, 255])

    # Converts the image from BGR to type HSV
    image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    image_HSV_specific = cv2.inRange(
        image_HSV, minimum_values, maximum_values)

    blur_image = cv2.blur(image_HSV_specific, (8, 8))  # kernal of 8x8 used

    # Finds contours in image (which can be used to identify shapes)
    contours, _ = cv2.findContours(
        blur_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(
        contours, key=lambda x: cv2.contourArea(x), reverse=True)

    # each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.
    for contour in contours:
        shape_area = cv2.contourArea(contour)

        if shape_area > 2400 and shape_area < 160000:
            perimeter = cv2.arcLength(contour, True)

            # accuracy parameter - maximum distance from contour to approximated contour
            episilon = 0.03 * perimeter
            verticies = cv2.approxPolyDP(
                contour, episilon, True)  # Adjust values as needed

            # Circles will still have verticies. If we can find a good balance then we are solid
            if len(verticies) > 6:

                # Gets parameters for a rectangle around object
                x, y, width, height = cv2.boundingRect(verticies)

                object_center_coordinates = [x + width/2, y - height/2]
                object_rectangle_width_height = [width, height]

                if x+(width/2) < video_frame_width/2:  # Left
                    cv2.putText(image, "Left", (x, y), cv2.FONT_HERSHEY_COMPLEX,
                                1, (255, 255, 255), 3, cv2.LINE_AA)
                elif x+(width/2) > video_frame_width:  # Right
                    cv2.putText(image, "Right", (x, y), cv2.FONT_HERSHEY_COMPLEX,
                                1, (255, 255, 255), 3, cv2.LINE_AA)
                else:
                    cv2.putText(image, "Center", (x, y), cv2.FONT_HERSHEY_COMPLEX,
                                1, (255, 255, 255), 3, cv2.LINE_AA)

                cv2.rectangle(image, (x, y), (x + width, y + height),
                              (0, 255, 0), 3)  # Draws rectangle on image

                # Exits for loop (and function) after the ball is found to reduce execution time
                cv2.imshow("Webcam_Input", image)
                print("area", shape_area)
                print("center", object_center_coordinates)
                print("width/height", object_rectangle_width_height)
                return shape_area, object_center_coordinates, object_rectangle_width_height
    return -1, [-1, -1], [-1, -1]


while True:
    success, frame = video_capture.read()
    frameCopy = frame.copy()
    object_contour_area, object_center_coordinates, object_rectangle_width_height = detect_object(success,
                                                                                                  frameCopy)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cv2.waitKey(10)

video_capture.release()
print("Terminated")


"""
Next steps:
- Find out what the x-coordinate is for the center of the image.
- Right now, the program is only detecting the ball every few frames, 
  increasing the reliability of the detection algorithm is important.
- Double check code for determining left and right relative positions (and provide actual values)
"""
