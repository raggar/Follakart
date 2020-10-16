import pid # Imports PID Class
import RPi.GPIO as GPIO

# We will need to test values to find the optimum ones
angle_pid = pid.PID(0.3, 0.03, 0.01, 100, 100) # Constructs PID class for when the robot needs to rotate
forward_pid = pid.PID(0.3, 0.03, 0.01, 100, 100) # Constructs PID class for when the robot needs to move forwards and backwards

desired_angle = 0 # We want the ball to be at 0 degrees relative to the centre
desired_distance = 10; # DECIDE THIS

# Define ports for motor 1
motor_1_forward = 24
motor_1_backward = 23
motor_1_en = 25

# Define ports for motor 2
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

pwm_motor_1.start(75)
pwm_motor_2.start(50)

# Tara and Amanda's functions apply in next two lines
left_or_right = 1 # 0 for Left, 1 for Right, 2 for centre
current_angle = 1 # This value will be given by Amanda and Tara's function
current_distance = 1 # This value will be given by Amanda and Tara's function

print("Starting")
while True:
    x=raw_input()
    
    GPIO.output(motor_1_forward,GPIO.HIGH)
    GPIO.output(motor_1_backward,GPIO.LOW)
    GPIO.output(motor_2_forward,GPIO.HIGH)
    GPIO.output(motor_2_backward,GPIO.LOW)

    # Uses a different PID depending if the car needs to rotate or move forward/backwards
    if current_angle > 10: # Rough value. Need some tolerance where it is acceptable to say the ball is "dead ahead"
        motor_pwm = angle_pid.get_output(current_angle, desired_angle)
    else:
        motor_pwm = forward_pid.get_output(current_distance, desired_distance)
    
    if motor_pwm > 0:
        if left_or_right == 0: # Left
            a = 0
            # power right motor using PWM
            # power left motor at about 5-10%
        elif left_or_right == 1: # Right
            a = 0
            # power left motor using PWM
            # power right motor at about 5-10%
        elif left_or_right == 2: # Centre
            a = 0
            # power both motors using PWM
    elif motor_pwm < 0:
        if left_or_right == 0:  # Left
            a = 0
            # power right motor using PWM
            # power left motor at about 5-10%
        elif left_or_right == 1:  # Right
            a = 0
            # power left motor using PWM
            # power right motor at about 5-10%
        elif left_or_right == 2:  # Centre
            a = 0
            # power both motors using PWM
            
    if x == 'e':
        GPIO.output(motor_1_forward,GPIO.LOW)
        GPIO.output(motor_1_backward,GPIO.LOW)
        GPIO.output(motor_2_forward,GPIO.LOW)
        GPIO.output(motor_2_backward,GPIO.LOW)
        print("Exiting")
        GPIO.cleanup()
        print("GPIO Clean up")
        print
        break;


""" END OF INFINITE WHILE LOOP """
