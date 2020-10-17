import pd  # Imports PID Class
import RPi.GPIO as GPIO
import time

# We will need to test values to find the optimum ones
angle_pid = pd.PD(5, 2, 60, 80)  # Constructs PID class for when the robot needs to rotate
forward_pid = pd.PD(0.4, 0.1, 40, 65)  # Constructs PID class for when the robot needs to move forwards and backwards

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

# Tara and Amanda's functions apply in next two lines
left_or_right = 1  # 0 for Left, 1 for Right, 2 for centre
current_angle = 26  # This value will be given by Amanda and Tara's function
current_distance = 1  # This value will be given by Amanda and Tara's function

print("Starting")
done = False
while not (done):

    # Uses a different PID depending if the car needs to rotate or move forward/backwards to say the ball is "dead ahead"
    motor_pwm = angle_pid.get_output(current_angle, desired_angle)

    current_angle = current_angle - 1

    if left_or_right == 0:  # Left
        pwm_motor_1.ChangeDutyCycle(25)
        pwm_motor_2.ChangeDutyCycle(motor_pwm)
        time.sleep(0.05)
        GPIO.output(motor_1_forward, GPIO.LOW)
        GPIO.output(motor_1_backward, GPIO.LOW)
        GPIO.output(motor_2_forward, GPIO.HIGH)
        GPIO.output(motor_2_backward, GPIO.LOW)
        # power right motor using PWM
        # power left motor at about 5-10%
    elif left_or_right == 1:  # Right
        pwm_motor_1.ChangeDutyCycle(motor_pwm)
        time.sleep(0.05)
        GPIO.output(motor_1_forward, GPIO.HIGH)
        GPIO.output(motor_1_backward, GPIO.LOW)
        GPIO.output(motor_2_forward, GPIO.LOW)
        GPIO.output(motor_2_backward, GPIO.LOW)
        # power left motor using PWM
        # power right motor at about 5-10%
    elif left_or_right == 2:  # Centre
        pwm_motor_1.ChangeDutyCycle(motor_pwm + 5)
        pwm_motor_2.ChangeDutyCycle(motor_pwm)
        time.sleep(0.05)
        GPIO.output(motor_1_forward, GPIO.HIGH)
        GPIO.output(motor_1_backward, GPIO.LOW)
        GPIO.output(motor_2_forward, GPIO.HIGH)
        GPIO.output(motor_2_backward, GPIO.LOW)
        print("Exiting")
        GPIO.cleanup()
        print("GPIO Clean up")
        print()
        # power both motors using PWM

    time.sleep(0.3)

    GPIO.output(motor_1_forward, GPIO.LOW)
    GPIO.output(motor_1_backward, GPIO.LOW)
    GPIO.output(motor_2_forward, GPIO.LOW)
    GPIO.output(motor_2_backward, GPIO.LOW)

    if (current_angle == 0):
        done = True

GPIO.cleanup()
print("GPIO Clean up")
""" END OF INFINITE WHILE LOOP """




