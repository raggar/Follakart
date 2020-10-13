import pid # Imports PID Class
""" Will write `import RPi.GPIO as IO` here """

# We will need to test values to find the optimum ones
angle_pid = pid.PID(0.3, 0.03, 0.00001, 100, 100) # Constructs PID class for when the robot needs to rotate
forward_pid = pid.PID(0.3, 0.03, 0.00001, 100, 100) # Constructs PID class for when the robot needs to move forwards and backwards

desired_angle = 0 # We want the ball to be at 0 degrees relative to the centre
desired_distance = 0; # DECIDE THIS

""" 
    Define all of the GPIO ports in use here.
    Since we are using PWM for the motors, we need to use GPIO 12 or 13
"""

""" Everything below this line will be in the infinite while loop!!! """

# Take image and process it

# Tara and Amanda's functions apply in next two lines
current_angle = 1 # This value will be given by Amanda and Tara's function
current_distance = 1 # This value will be given by Amanda and Tara's function

if current_angle > 10: # Rough value. Need some tolerance where it is acceptable to say the ball is "dead ahead"
    motor_pwm = angle_pid.get_output(current_angle, desired_angle)
else:
    motor_pwm = forward_pid.get_output(current_distance, desired_distance)

"""
    Pseudocode:
    
    if ball is to left:
        power right motor using PWM
        power left motor at about 5-10%
        
    else if ball is to right:
        power left motor using PWM
        power right motor at about 5-10%
        
    else if ball is straight (current_angle > smth):
        power both motors using PWM
    
    
    
    delay for a couple milliseconds
    
    ***END OF LOOP***
            
"""



