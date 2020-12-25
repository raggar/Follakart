# Follakart (Software Engineering 2020 Project)

Follakart is an autonomous car that will track and follow a predefined object. Powered using Raspberry Pi, a camera module, and DC motors, Follakart will analyze video frames to locate an object (hereby referred to as the “tracked object”) and position itself to keep the tracked object near the centre of the camera frame. 

## Components

### Hardware
The cart is powered by a raspberry pi, picamera, and two motors that are all fitted within a compact frame. Early development featured unit testing to ensure each component worked as intended. In later stages, wiring all components together and creating the aesthetic exterior chassis took precedence. It took two iterations, but our current model’s wiring is better organized and the layout puts more weight at the back, improving traction. 

### Computer Vision
To consistently and adequately track our desired object, we used computer vision, specifically the OpenCV library in Python. Starting off, we experimented with several different methods of tracking such as shape, silhouette and kernel tracking. Upon weighing each method's pros and cons we settled upon using HSV or colour detection, and thus began by collecting the corresponding saturation, and hue values of our object. Once calibrated, we developed the primary algorithm which encompassed sending each frame of video input to a function, that would then, through the use of opencv gradience and blurring effects, create contours around the objects edges. Drawing a rectangle around these contours is what then allowed us to determine the object’s area, x,y coordinates and position relative to the center line. Finally, we spent a lot of time testing and adjusting fixed values to improve the algorithm’s efficiency as much as possible and account for different lightings and environments. This was an essential step as the effectiveness of the OpenCV algorithm, just like the other components, is crucial to the overall vision and execution of Follakart. 

### Object Position and Motor control
After detecting the object, the cart calculates the distance and angle between itself and the object using algorithms whose errors were halved over the past month.
Once detected, the cart powers its motors to either rotate or translate as needed. Once this motion was developed, we integrated a PD controller and Pulse Width Modulation to ensure the motors get the optimum amount of power. 
Additionally, if the object leaves the frame, the cart triggers a macro which was improved to leverage past object-position data to locate the object faster by varying rotation direction and speed. 


## General Notes

### Pip

Pip is the reference Python package manager. It’s used to install and update packages. You’ll need to make sure you have the latest version of pip installed.

`py -m pip install --upgrade pip`

To install packages run

`pip install package-name`

### Virtual Env

Virtualenv is used to manage Python packages for different projects. Using virtualenv allows you to avoid installing Python packages globally which could break system tools or other projects. You can create a virtual enviornment by running the following commands:

Mac:
`python3 -m venv env`

Windows:
`py -m venv env`

To then activate your virtual enviornment run

Mac:
`source env/bin/activate`

Windows:
`.\env\Scripts\activate`

To leave the virtual enviornment type `deactivate` in your terminal/command prompt

### Requirements.txt

A requirements.txt file acts as a package.json file and basically lists each dependency used in the project. Thus, when pulling from master ensure that you run the following command to update your dependencies

`pip install -r requirements.txt`

Make sure that when you install new project packages that they are added to the requirements.txt file (should be kept at the root directory). Use `pip freeze` to view a list of packages.

### References
- Computer Vision :
    -  [https://www.youtube.com/watch?v=N81PCpADwKQ&t=2017s](https://www.youtube.com/watch?v=N81PCpADwKQ&t=2017s7)
        - This video was used to learn how to install opencv, detect HSV values and draw contours
    - [https://docs.opencv.org/master/](https://docs.opencv.org/master/)
    -   - Used OpenCV documentation to learn the fundamentals of computer vision 
- Object Position Calculations:
    -  [https://gist.github.com/bbartling/a9a64d8dd176c89575b89bab2ed23ae7](https://gist.github.com/bbartling/a9a64d8dd176c89575b89bab2ed23ae7)
        - This code was used to learn methods of calculating the distance between a camera and an object
    - [https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/](https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/)
        - This website was used to examine a method of calcultaing the distance between a camera and an object
    - [https://stackoverflow.com/questions/17499409/opencv-calculate-angle-between-camera-and-pixel/17505081](https://stackoverflow.com/questions/17499409/opencv-calculate-angle-between-camera-and-pixel/17505081)
        - This website was used to find issues people had while calculating the angle and ensure we did not run into the same problems
- Motors and GPIO:
    - [https://howchoo.com/g/mjg5ytzmnjh/controlling-dc-motors-using-your-raspberry-pi](https://howchoo.com/g/mjg5ytzmnjh/controlling-dc-motors-using-your-raspberry-pi)
        - This link was used to learn how to initialize and set the RPi GPIO pins using the RPi.GPIO Library. 
    - [https://www.mbtechworks.com/projects/raspberry-pi-pwm.html](https://www.mbtechworks.com/projects/raspberry-pi-pwm.html)
        - This link was used to learn how to integrate Pulse Width Modulation (PWM) using the RPi.GPIO Library. 
