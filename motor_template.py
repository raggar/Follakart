import pid # Imports PID Class
""" Will write `import RPi.GPIO as GPIO` here """

# We will need to test values to find the optimum ones
angle_pid = pid.PID(0.3, 0.03, 0.01, 100, 100) # Constructs PID class for when the robot needs to rotate
forward_pid = pid.PID(0.3, 0.03, 0.01, 100, 100) # Constructs PID class for when the robot needs to move forwards and backwards

desired_angle = 0 # We want the ball to be at 0 degrees relative to the centre
desired_distance = 0; # DECIDE THIS

""" 
    Define all of the GPIO ports in use here.
    Since we are using PWM for the motors, we need to use GPIO 12 or 13
"""

""" START OF INFINITE WHILE LOOP """

# Take image and process it

# Tara and Amanda's functions apply in next two lines
left_or_right = 1 # 0 for Left, 1 for Right, 2 for centre
current_angle = 1 # This value will be given by Amanda and Tara's function
current_distance = 1 # This value will be given by Amanda and Tara's function

# Uses a different PID depending if the car needs to rotate or move forward/backwards
if current_angle > 10: # Rough value. Need some tolerance where it is acceptable to say the ball is "dead ahead"
    motor_pwm = angle_pid.get_output(current_angle, desired_angle)
else:
    motor_pwm = forward_pid.get_output(current_distance, desired_distance)

if motor_pwm > 0:
    if left_or_right == 0: # Left
        # power right motor using PWM
        # power left motor at about 5-10%
    elif left_or_right == 1: # Right
        # power left motor using PWM
        # power right motor at about 5-10%
    elif left_or_right == 2: # Centre
        # power both motors using PWM
elif motor_pwm < 0:
    if left_or_right == 0:  # Left
        # power right motor using PWM
        # power left motor at about 5-10%
    elif left_or_right == 1:  # Right
        # power left motor using PWM
        # power right motor at about 5-10%
    elif left_or_right == 2:  # Centre
        # power both motors using PWM

# Delay for 10-100 ms

""" END OF INFINITE WHILE LOOP """



