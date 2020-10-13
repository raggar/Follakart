import time

# Creates class object for the PID controller
class PID:

    """
        Parameters:
            @param kproportional = proportional gain
            @param kintegral = integral gain
            @param kderivative  = dereivative gainn
            @param maximum_integral = maximum magnitude of integral to prevent integral windup
            @param maximum_output = maximum PWM output possible

            @return => void function
    """
    def __init__(self, kproportional, kintegral, kderivative, maximum_integral, maximum_output):
        self.kp = kproportional
        self.ki = kintegral
        self.kd = kderivative
        self.integral = 0
        self.max_integral = maximum_integral
        self.min_out = 0
        self.max_out = maximum_output
        self.past_positions = [0, 0] # Stores past positions of the object for derivative calculations
        self.measurement_time = [time.time(), 0]
        # print("PID Controller Constructed") # Debugging purposes

    """
        Parameters:
            @param current = current value between ball and car
            @param desired = deisred value

            @return = PWM value for motors (motor power)
    """
    def get_output(self, current, desired):
        # print("Getting output") # Debugging purposes

        error = desired - current

        # Determines derivative value
        self.past_positions[1] = self.past_positions[0]
        self.past_positions[0] = current

        self.measurement_time[1] = self.measurement_time[0]
        self.measurement_time[0] = time.time()

        # print(str(self.measurement_time[1]))

        derivative = (self.past_positions[0] - self.past_positions[1])/(self.measurement_time[0] - self.measurement_time[1])

        """
            If the integral becomes too large in either direction, it may cause the car to overshoot its target. 
            Thus, to prevent this, we ensure the integral attains a maximum magnitude. 
        """

        # Determines integral value
        self.integral = self.integral + error

        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < self.max_integral * -1:
            self.integral = self.max_integral * -1

        # Calculates required motor output and ensures it stays within min and max values
        motor_output = error * self.kp + self.integral * self.ki + derivative * self.kd

        if motor_output < self.min_out:
            motor_output = self.min_out
        elif motor_output > self.max_out:
            motor_output = self.max_out

        # print("Values: ", str(error), "; ", str(self.integral), "; ", str(derivative))
        # print("Output: ", str(motor_output))

        return motor_output # Returns the required PWM value



