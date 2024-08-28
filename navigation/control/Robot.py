import time
import gpiozero
import math
from State import State

WHEEL_SEP = 227/1000
WHEEL_RAD = 61/2/1000
CPR = 48
GEAR_RATIO = 74.38

"""
1. how to remember order of operations? Keep them in a tray as memory?
2. how to return back to start location?

"""

class Robot:    

    def __init__(self, state_init) -> None:
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

        self.scaling = {
            "ROTATE": 3.7,
            "FORWARD": 4.2,
            "BACK": 1
        }

        self.state = state_init

        self.wheel_radius = WHEEL_RAD

        self.motor1_multiplier = 1
        self.motor2_multiplier = 1 

        self.motor1_pwm = gpiozero.PWMOutputDevice(pin=self.pins["PIN_MOTOR1_PWM_ENABLE"],active_high=True,initial_value=0,frequency=100)
        self.motor1_in1 = gpiozero.OutputDevice(pin=self.pins["PIN_MOTOR1_IN1"])
        self.motor1_in2 = gpiozero.OutputDevice(pin=self.pins["PIN_MOTOR1_IN2"])
        self.motor1_encoder = gpiozero.RotaryEncoder(a=self.pins["PIN_MOTOR1_A_OUT"], b=self.pins["PIN_MOTOR1_B_OUT"],max_steps=100000) 

        self.motor2_pwm = gpiozero.PWMOutputDevice(pin=self.pins["PIN_MOTOR2_PWM_ENABLE"],active_high=True,initial_value=0,frequency=100)
        self.motor2_in1 = gpiozero.OutputDevice(pin=self.pins["PIN_MOTOR2_IN1"])
        self.motor2_in2 = gpiozero.OutputDevice(pin=self.pins["PIN_MOTOR2_IN2"])
        self.motor2_encoder = gpiozero.RotaryEncoder(a=self.pins["PIN_MOTOR2_A_OUT"], b=self.pins["PIN_MOTOR2_B_OUT"],max_steps=100000) 

        self.reset_position(0.0,0.0,0.0)
        self.reset_encoders()

    def reset_position(self, x, y, th):
        self.x = x
        self.y = y
        self.th = th

    def reset_encoders(self):
        self.motor1_encoder.steps = 0
        self.motor2_encoder.steps = 0

    def distance_per_step(self):
        wheel_rotatations_per_step = 1 / (CPR*GEAR_RATIO)
        return 2*math.pi*self.wheel_radius*wheel_rotatations_per_step
    
    def move_forward_alt(self, distance, speed=0.5):
        """
        Moves the robot forward by a specific distance at the given speed.
        """
        encoder_steps = self.dist_to_encoder_steps(distance)
        self.reset_encoders()

        # Set motor direction for forward movement
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.on()
        self.motor2_in2.off()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        # Start moving forward
        while abs(self.motor1_encoder.steps) < encoder_steps or abs(self.motor2_encoder.steps) < encoder_steps:

            time.sleep(0.01)  # Small delay for sensor feedback

        # Stop motors
        self.motor1_pwm.off()
        self.motor2_pwm.off()
        print("done\n")

    def turn_off_all(self):
        self.motor1_pwm.off()
        self.motor1_in1.off()
        self.motor1_in2.off()

        self.motor2_pwm.off()
        self.motor1_in1.off()
        self.motor1_in2.off()

    """MISSION METHODS"""
    def start_rotating_clockwise(self, speed):
        self.reset_encoders()
        
        # Set motor directions for clockwise rotation
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.off()
        self.motor2_in2.on()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

    def update_orientation(self, angle_deg):
        new_angle = (self.th + angle_deg) % 360

        # # Adjust to be within -180 to 180 range
        # if new_angle > 180:
        #     new_angle -= 360
        # elif new_angle <= -180:
        #     new_angle += 360
        self.th = new_angle

    def update_x(self, x_diff):
        self.x = self.x + x_diff
    
    def update_y(self, y_diff):
        self.y = self.y + y_diff

    def stop_rotating_clockwise(self):
        
        self.motor1_pwm.off()
        self.motor2_pwm.off()

        self.motor1_in1.off()
        self.motor1_in2.off()
        self.motor2_in1.off()
        self.motor2_in2.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        angle_deg = self.encoder_steps_to_angle(motor2_steps)

        self.update_orientation(0-angle_deg)
        self.reset_encoders()
    
    def stop_rotating_anticlockwise(self):

        self.motor1_pwm.off()
        self.motor2_pwm.off()

        self.motor1_in1.off()
        self.motor1_in2.off()
        self.motor2_in1.off()
        self.motor2_in2.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        angle_deg = self.encoder_steps_to_angle(motor1_steps)

        self.update_orientation(angle_deg)
        self.reset_encoders()

    def start_forward(self, speed):
        self.reset_encoders()

        # Set motor direction for forward movement
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.on()
        self.motor2_in2.off()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed 

    def stop_forward(self):
        self.motor1_in1.off()
        self.motor2_in1.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        dist_travelled = self.encoder_steps_to_dist(max(motor1_steps, motor2_steps))

        """check odometry calcs"""

        # self.update_x(x_diff)
        # self.update_y(y_diff)

        self.update_position(dist_travelled)

        self.reset_encoders()


    def update_position(self, dist_travelled):
        angle_deg = self.th

        print(angle_deg)

        angle_rad = math.radians(self.th)
        if 0 <= angle_deg < 90:
            x_diff = math.cos(angle_rad)*dist_travelled
            y_diff = math.sin(angle_rad)*dist_travelled

            print("1")
            self.update_x(0-x_diff)
            self.update_y(y_diff)   

        elif 90 <= angle_deg < 180:
            angle = 180-angle_deg
            print("2")
            x_diff = math.cos(angle_rad)*dist_travelled
            y_diff = math.sin(math.radians(angle))*dist_travelled

            self.update_x(x_diff)
            self.update_y(y_diff)  

        elif 180 <= angle_deg < 270:
            angle = 270-angle_deg

            print("3")
            x_diff = math.sin(math.radians(angle))*dist_travelled
            y_diff = math.cos(math.radians(angle))*dist_travelled 

            self.update_x(x_diff)
            self.update_y(y_diff)  

        else:
            angle = 360-angle_deg

            print("4")
            x_diff = math.cos(math.radians(angle))*dist_travelled
            y_diff = math.sin(math.radians(angle))*dist_travelled 

            self.update_x(0-x_diff)
            self.update_y(0-y_diff)  

   


    def start_backward(self, speed):
        self.reset_encoders()

        # Set motor direction for forward movement
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.on()
        self.motor2_in2.off()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed
        
        """check odometry calcs"""

       

        self.reset_encoders()


    
    def stop_backwards(self):
        self.motor1_pwm.off()
        self.motor2_pwm.off()

        self.motor1_in2.off()
        self.motor2_in2.off()

    
    def encoder_steps_to_angle(self, encoder_steps):
        arc_len = self.encoder_steps_to_dist(encoder_steps)
        angle_rad = self.arc_len_to_angle(arc_len)
        angle_deg = math.degrees(angle_rad)
        return angle_deg
    
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

        time.sleep(duration)

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

    def move(self, distance, speed):
        if distance > 0: 
            self.move_forward(distance, speed)
        else:
            self.move_backward(distance, speed)



    def move_forward(self, distance, speed=0.5):
        """
        Moves the robot forward by a specific distance at the given speed.
        """
        encoder_steps = self.dist_to_encoder_steps(distance)/4
        left_encoder_steps = encoder_steps/0.9
        right_encoder_steps = encoder_steps
       
        self.reset_encoders()

        # Kp_forward = 0.01  # Proportional gain for forward adjustment

        # Set motor direction for forward movement
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.on()
        self.motor2_in2.off()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        # Start moving forward
        while abs(self.motor1_encoder.steps) < encoder_steps or abs(self.motor2_encoder.steps) < encoder_steps:

            # if not (abs(self.motor1_encoder.steps) < left_encoder_steps):
            #     self.motor1_pwm.off()
            
            # if not abs(self.motor2_encoder.steps) < right_encoder_steps:
            #     self.motor2_pwm.off()

            # # Calculate the difference in encoder steps
            # error = self.motor1_encoder.steps - self.motor2_encoder.steps
            # print(error)

            # # # Adjust PWM values for forward movement
            # self.motor1_pwm.value = max(0, min(1, speed - Kp_forward * error))
            # self.motor2_pwm.value = max(0, min(1, speed + Kp_forward * error))
            # print(f"left = {self.motor1_pwm.value}       right = {self.motor2_pwm.value}")

            time.sleep(0.01)  # Small delay for sensor feedback
    


        # Stop motors
        self.motor1_pwm.off()
        self.motor2_pwm.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        dist_travelled = self.encoder_steps_to_dist(max(motor1_steps, motor2_steps))

        """check odometry calcs"""

        # self.update_x(x_diff)
        # self.update_y(y_diff)

        self.update_position(dist_travelled, scaling_factor = self.scaling["FORWARD"])

        self.reset_encoders()

        print("done\n")

    def move_backward(self, distance, speed=0.5):
        """
        Moves the robot backward by a specific distance at the given speed.
        """
        encoder_steps = self.dist_to_encoder_steps(distance)/3.3
        self.reset_encoders()

        # Kp_backward = 0.01  # Proportional gain for backward adjustment

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
            # error = self.motor1_encoder.steps - self.motor2_encoder.steps

            # # Adjust PWM values for backward movement
            # self.motor1_pwm.value = max(0, min(1, speed + Kp_backward * error))
            # self.motor2_pwm.value = max(0, min(1, speed - Kp_backward * error))

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

    def encoder_steps_to_dist(self, encoder_steps):
        wheel_rotations = encoder_steps/(CPR* GEAR_RATIO)
        dist = wheel_rotations*(2 * math.pi * self.wheel_radius) 
        return dist

    def arc_len_to_angle(self, arc_len):
        angle_rad  = arc_len /(WHEEL_SEP/2)
        return angle_rad

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

    def rotate(self, angle_degrees, speed):
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
            self.rotate_clockwise(angle_degrees/4.2, speed)
        else:
            self.rotate_anticlockwise(angle_degrees/4.2, speed)


    def rotate_clockwise(self, angle, speed=0.5):
        """Rotates the robot clockwise by a specific angle at the given speed."""
        encoder_steps = self.calculate_encoder_steps_for_rotation(abs(angle))
        self.reset_encoders()

        # Kp_rotate = 0.01  # Proportional gain for rotation adjustment

        # Set motor directions for clockwise rotation
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.off()
        self.motor2_in2.on()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        while abs(self.motor1_encoder.steps) < encoder_steps or abs(self.motor2_encoder.steps) < encoder_steps:
            # error = self.motor1_encoder.steps + self.motor2_encoder.steps
            # self.motor1_pwm.value = max(0, min(1, speed - Kp_rotate * error))
            # self.motor2_pwm.value = max(0, min(1, speed - Kp_rotate * error))
            time.sleep(0.01)



        self.motor1_pwm.off()
        self.motor2_pwm.off()

        self.motor1_in1.off()
        self.motor1_in2.off()
        self.motor2_in1.off()
        self.motor2_in2.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        angle_deg = self.encoder_steps_to_angle(motor2_steps)

        self.update_orientation(0-angle_deg)
        self.reset_encoders()

        

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
            # error = self.motor1_encoder.steps + self.motor2_encoder.steps
            # self.motor1_pwm.value = max(0, min(1, speed - Kp_rotate * error))
            # self.motor2_pwm.value = max(0, min(1, speed - Kp_rotate * error))
            time.sleep(0.01)


        self.motor1_pwm.off()
        self.motor2_pwm.off()

        self.motor1_in1.off()
        self.motor1_in2.off()
        self.motor2_in1.off()
        self.motor2_in2.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        angle_deg = self.encoder_steps_to_angle(motor2_steps)

        self.update_orientation(angle_deg)
        self.reset_encoders()


    # def handle(self, frame):

    #     match self.state:
    #         case State.START:
    #             # check if any balls detected in current frame 

                
                
    #         case State.

    #             if 

            
    #         case State.EXPLORE_CENTRE:

            
    #         case State.FOUND_TARGET:

            
    #         case State.ON_TARGET:
       

     



    

        
  