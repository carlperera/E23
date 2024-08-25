import gpiozero
import math

from time import sleep
from time import time

WHEEL_SEP = (147 + 64)/1000
WHEEL_RAD = (53/2)/1000
CPR = 48
GEAR_RATIO = 74.38

class Robot:

    def __init__(self) -> None:
        self.pins = {
            "PIN_MOTOR1_IN1":         17,
            "PIN_MOTOR1_IN2":         27, 
            "PIN_MOTOR1_PWM_ENABLE":  18, 
            "PIN_MOTOR1_A_OUT":       21,
            "PIN_MOTOR1_B_OUT":       20,
            "PIN_MOTOR2_IN1":         23,
            "PIN_MOTOR2_IN2":         24,
            "PIN_MOTOR2_PWM_ENABLE":   9, 
            "PIN_MOTOR2_A_OUT":       14,
            "PIN_MOTOR2_B_OUT":       15
        }

        self.wheel_radius = WHEEL_RAD

        self.motor1_multiplier = 1
        self.motor2_multiplier = 1 

        self.motor1_pwm = gpiozero.PWMOutputDevice(pin=self.pins.PIN_MOTOR1_PWM_ENABLE,active_high=True,initial_value=0,frequency=100)
        self.motor1_in1 = gpiozero.OutputDevice(pin=self.pins.PIN_MOTOR1_IN1)
        self.motor1_in2 = gpiozero.OutputDevice(pin=self.pins.PIN_MOTOR1_IN2)
        self.motor1_encoder = gpiozero.RotaryEncoder(a=self.pins.PIN_MOTOR1_A_OUT, b=self.pins.PIN_MOTOR1_B_OUT,max_steps=100000) 

        self.motor2_pwm = gpiozero.PWMOutputDevice(pin=self.pins.PIN_MOTOR2_PWM_ENABLE,active_high=True,initial_value=0,frequency=100)
        self.motor2_in1 = gpiozero.OutputDevice(pin=self.pins.PIN_MOTOR2_IN1)
        self.motor2_in2 = gpiozero.OutputDevice(pin=self.pins.PIN_MOTOR2_IN2)
        self.motor2_encoder = gpiozero.RotaryEncoder(a=self.pins.PIN_MOTOR2_A_OUT, b=self.pins.PIN_MOTOR2_B_OUT,max_steps=100000) 

        self.reset_encoders()

    def reset_encoders(self):
        self.motor1_encoder.steps = 0
        self.motor2_encoder.steps = 0

    def distance_per_step(self):
        wheel_rotatations_per_step = 1 / (CPR*GEAR_RATIO)
        return 2*math.pi*self.wheel_radius*wheel_rotatations_per_step
    
    def calibrate_motors(self, duration = 5):
        """
        Runs both motors at full speed for a fixed duration and calculates the ratio
        of their speeds based on encoder feedback.
        """
        self.reset_encoders()
        
        # Run both motors at full speed forwards direction 
        self.motor1_pwm.value = 1.0
        self.motor2_pwm.value = 1.0
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.on()
        self.motor2_in2.off()

        sleep(duration)

        # Stop motors
        self.motor1_pwm.off()
        self.motor2_pwm.off()

        # Calculate encoder steps difference
        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        # Determine multipliers to equalize wheel speeds
        if motor1_steps != 0 and motor2_steps != 0:
            self.motor1_multiplier = motor2_steps / motor1_steps
            self.motor2_multiplier = motor1_steps / motor2_steps
        else:
            print("Error: Encoder readings are zero. Check connections and sensors.")

        print(f"Calibration complete. Motor1 Multiplier: {self.motor1_multiplier}, Motor2 Multiplier: {self.motor2_multiplier}")


    def move_forward(self, distance, speed=0.5):
        """
        Moves the robot forward by a specific distance at the given speed.
        """
        encoder_steps = self.dist_to_encoder_steps(distance)
        self.reset_encoders()

        Kp_forward = 0.01  # Proportional gain for forward adjustment

        # Set motor direction for forward movement
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.on()
        self.motor2_in2.off()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        # Start moving forward
        while abs(self.motor1_encoder.steps) < encoder_steps or abs(self.motor2_encoder.steps) < encoder_steps:
            # Calculate the difference in encoder steps
            error = self.motor1_encoder.steps - self.motor2_encoder.steps

            # Adjust PWM values for forward movement
            self.motor1_pwm.value = max(0, min(1, speed - Kp_forward * error))
            self.motor2_pwm.value = max(0, min(1, speed + Kp_forward * error))

            time.sleep(0.01)  # Small delay for sensor feedback

        # Stop motors
        self.motor1_pwm.off()
        self.motor2_pwm.off()

    def move_backward(self, distance, speed=0.5):
        """
        Moves the robot backward by a specific distance at the given speed.
        """
        encoder_steps = self.dist_to_encoder_steps(distance)
        self.reset_encoders()

        Kp_backward = 0.01  # Proportional gain for backward adjustment

        # Set motor direction for backward movement
        self.motor1_in1.off()
        self.motor1_in2.on()
        self.motor2_in1.off()
        self.motor2_in2.on()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        # Start moving backward
        while abs(self.motor1_encoder.steps) < encoder_steps or abs(self.motor2_encoder.steps) < encoder_steps:
            # Calculate the difference in encoder steps
            error = self.motor1_encoder.steps - self.motor2_encoder.steps

            # Adjust PWM values for backward movement
            self.motor1_pwm.value = max(0, min(1, speed + Kp_backward * error))
            self.motor2_pwm.value = max(0, min(1, speed - Kp_backward * error))

            time.sleep(0.01)  # Small delay for sensor feedback

        # Stop motors
        self.motor1_pwm.off()
        self.motor2_pwm.off()



    def move_forward_at_speed_for_time(self, base_speed, duration):
        """
        Moves the robot forward at the specified base speed for the specified duration,
        dynamically adjusting the motor speeds to keep the robot moving straight.
        """
        self.reset_encoders()

        Kp = 0.01  # Proportional gain for adjustment

        self.motor1_pwm.value = base_speed
        self.motor2_pwm.value = base_speed
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.on()
        self.motor2_in2.off()

        start_time = time()
        while time() - start_time < duration:
            # Calculate the difference in encoder steps
            error = self.motor1_encoder.steps - self.motor2_encoder.steps
            
            # Adjust the PWM values based on the error
            self.motor1_pwm.value = max(0, min(1, base_speed - Kp * error))
            self.motor2_pwm.value = max(0, min(1, base_speed + Kp * error))

            time.sleep(0.1)  # Small delay for sensor feedback

        # Stop motors
        self.motor1_pwm.off()
        self.motor2_pwm.off()
    

    def dist_to_encoder_steps(self, dist):
        wheel_rotations = dist/ (2 * math.pi * self.wheel_radius)
        encoder_steps = int(wheel_rotations * CPR * GEAR_RATIO)
        return encoder_steps

    def calculate_encoder_steps_for_rotation(self, angle_degrees):
        """
        Calculate the required encoder steps for each motor to rotate the robot by a specific angle.
        """
        # Convert angle to radians
        angle_radians = math.radians(angle_degrees)
        
        # Calculate arc length that each wheel needs to travel for the rotation
        arc_length = (angle_radians * WHEEL_SEP/2) # arc_length = theta_rad*radius

        encoder_steps = self.dist_to_encoder_steps(arc_length)
        
        return encoder_steps
   
    def rotate(self, angle_degrees, speed=0.5):
        """
        Rotates the robot by a specific angle (in degrees) at the given speed.

        -> for a turn in place (robot rotating around its centre) the radius is half the separation distance 
            -> WHEEL_SEP/2
        -> arc length for desired turn: 
            -> L = theta*WHEEL_SEP/2
        -> arc length into wheel rotations
            -> rotations = L/(2*pi*WHEEL_RAD)
        Args:
            :
        """
        encoder_steps = self.calculate_encoder_steps_for_rotation(angle_degrees)
        
        self.reset_encoders()

        # Determine direction based on the sign of the angle
        if angle_degrees > 0:
            # Turn right: Motor 1 forward, Motor 2 backward
            self.motor1_in1.on()
            self.motor1_in2.off()
            self.motor2_in1.off()
            self.motor2_in2.on()
        else:
            # Turn left: Motor 1 backward, Motor 2 forward
            self.motor1_in1.off()
            self.motor1_in2.on()
            self.motor2_in1.on()
            self.motor2_in2.off()

        # Set PWM values
        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        # Rotate until the required number of steps is reached
        while abs(self.motor1_encoder.steps) < encoder_steps or abs(self.motor2_encoder.steps) < encoder_steps:

            time.sleep(0.01)  # Small delay for sensor feedback


        # Stop motors
        self.motor1_pwm.off()
        self.motor2_pwm.off()



    def rotate_clockwise(self, angle, speed=0.5):
        """Rotates the robot clockwise by a specific angle at the given speed."""
        encoder_steps = self.calculate_encoder_steps_for_rotation(abs(angle))
        self.reset_encoders()

        Kp_rotate = 0.01  # Proportional gain for rotation adjustment

        # Set motor directions for clockwise rotation
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.off()
        self.motor2_in2.on()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        while abs(self.motor1_encoder.steps) < encoder_steps or abs(self.motor2_encoder.steps) < encoder_steps:
            error = self.motor1_encoder.steps + self.motor2_encoder.steps
            self.motor1_pwm.value = max(0, min(1, speed - Kp_rotate * error))
            self.motor2_pwm.value = max(0, min(1, speed - Kp_rotate * error))
            time.sleep(0.01)

        self.motor1_pwm.off()
        self.motor2_pwm.off()


    def rotate_anticlockwise(self, angle, speed=0.5):
        """Rotates the robot counterclockwise by a specific angle at the given speed."""
        encoder_steps = self.calculate_encoder_steps_for_rotation(abs(angle))
        self.reset_encoders()

        Kp_rotate = 0.01  # Proportional gain for rotation adjustment

        # Set motor directions for counterclockwise rotation
        self.motor1_in1.off()
        self.motor1_in2.on()
        self.motor2_in1.on()
        self.motor2_in2.off()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        while abs(self.motor1_encoder.steps) < encoder_steps or abs(self.motor2_encoder.steps) < encoder_steps:
            error = self.motor1_encoder.steps + self.motor2_encoder.steps
            self.motor1_pwm.value = max(0, min(1, speed - Kp_rotate * error))
            self.motor2_pwm.value = max(0, min(1, speed - Kp_rotate * error))
            time.sleep(0.01)

        self.motor1_pwm.off()
        self.motor2_pwm.off()


    def rotate(self, angle_deg, speed = 0.5):
        if (angle_deg > 0): # clockwise 
            self.rotate_clockwise(angle=angle_deg, speed=speed)
        else:
            self.rotate_anticlockwise(angle=abs(angle_deg), speed=speed)

    

        
  