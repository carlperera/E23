import time
import gpiozero
import math
from State import State
from Vision import Vision
from State import StartPosition
from Claw import Claw
from Flap import Flap

WHEEL_SEP = 227/1000
WHEEL_RAD = 61/2/1000
CPR = 48
GEAR_RATIO = 74.38

"""
1. how to remember order of operations? Keep them in a tray as memory?
2. how to return back to start location?
"""

from enum import Enum

class Scaling(Enum):
    ROTATE = 4.2  # 3.7
    FORWARD = 3.486  # 4.2
    BACK = 1.0

class Constants(Enum):
    ROTATE = 3 / 100
    FORWARD = 4.282 / 100
    TRAY_BALL_THRESHOLD = 4
    SIMULATION_DURATION_S = 10*60

class Speeds(Enum):
    MOVING_TO_TARGET = 1.0
    ROTATING_EXPLORE = 0.2
    CLOSE_TO_TARGET = 0.4
    ROTATE_TO_TARGET = 0.15
    RETURN_ROTATE = 0.8
    ROTATE_TO_CENTRE = 0.6
    RETREAT_BACK = 0.6
    MOVING_TO_CENTRE = 0.8 

class PINSMOTOR(Enum):
    PIN_MOTOR1_IN1 =         17
    PIN_MOTOR1_IN2 =          27
    PIN_MOTOR1_PWM_ENABLE =  18
    PIN_MOTOR1_A_OUT =      21
    PIN_MOTOR1_B_OUT =       20
    PIN_MOTOR2_IN1 =        23
    PIN_MOTOR2_IN2 =       24
    PIN_MOTOR2_PWM_ENABLE =  9
    PIN_MOTOR2_A_OUT =    14
    PIN_MOTOR2_B_OUT =     15

class RotateDirection(Enum):
    ANTICLOCKWISE = 0 
    CLOCKWISE = 1 

class Robot:    

    def __init__(self, vision: Vision, claw: Claw, start_pos: StartPosition) -> None:

        # ----------------  Specifications ----------------
        self.wheel_radius = WHEEL_RAD
        self.simulation_duration_s = 10*60 # the robot should stop after this time (KAPOOTY TIME)

        # ----------------  Calibration ----------------
        self.calibrated_close_dist = 0.3
        self.calibrated_retreat_dist = 0.3 
        self.calibrated_centre_move = 3.0
        self.left_motor_scale = 1.0
        self.motor1_multiplier = 1
        self.motor2_multiplier = 1 
        
        # ----------------  Start Position ----------------
        self.start_pos = start_pos
        if start_pos is StartPosition.LEFT:
            self.state = State.START_LEFT
        else:
            self.state = State.START_RIGHT

        self.close_to_target_calibration = 0.35

        # ----------------  ENCODER SETUP ----------------
        self.motor1_pwm = gpiozero.PWMOutputDevice(pin=PINSMOTOR.PIN_MOTOR1_PWM_ENABLE.value,active_high=True,initial_value=0,frequency=100)
        self.motor1_in1 = gpiozero.OutputDevice(pin=PINSMOTOR.PIN_MOTOR1_IN1.value)
        self.motor1_in2 = gpiozero.OutputDevice(pin=PINSMOTOR.PIN_MOTOR1_IN2.value)
        self.motor1_encoder = gpiozero.RotaryEncoder(a=PINSMOTOR.PIN_MOTOR1_A_OUT.value, b=PINSMOTOR.PIN_MOTOR1_B_OUT.value,max_steps=100000) 

        self.motor2_pwm = gpiozero.PWMOutputDevice(pin=PINSMOTOR.PIN_MOTOR2_PWM_ENABLE.value,active_high=True,initial_value=0,frequency=100)
        self.motor2_in1 = gpiozero.OutputDevice(pin=PINSMOTOR.PIN_MOTOR2_IN1.value)
        self.motor2_in2 = gpiozero.OutputDevice(pin=PINSMOTOR.PIN_MOTOR2_IN2.value)
        self.motor2_encoder = gpiozero.RotaryEncoder(a=PINSMOTOR.PIN_MOTOR2_A_OUT.value, b=PINSMOTOR.PIN_MOTOR2_B_OUT.value,max_steps=100000) 


        # ----------------  START ODOMETRY POSITION ----------------
        self.x = 0
        self.y = 0
        self.th = 0
        self.reset_position(0.0,0.0,0.0)
        self.reset_encoders()

        # ----------------  VISION ----------------
        self.vision = vision

        self.angle_to_rotate_to = None
        
        # ----------------  BALLS - TRAY ----------------
        self.tray_ball_threshold = Constants.TRAY_BALL_THRESHOLD # number of balls in robot tray at which point robot goes to collection box


        # ----------------  ----------------
        self.flap = Flap()
        self.claw = Claw()
  
    def shutdown(self):
        self.vision.close()

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

    def start_rotating_anticlockwise(self, speed):
        self.reset_encoders()
        
        # Set motor directions for clockwise rotation
        self.motor1_in1.off()
        self.motor1_in2.on()
        self.motor2_in1.on()
        self.motor2_in2.off()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed
        


    def update_orientation(self, angle_deg):
        new_angle = (self.th + angle_deg) % 360
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

        angle_raw= self.encoder_steps_to_angle(max(motor1_steps, motor2_steps))
        angle_deg = self.angle_scale_down(angle_raw)
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

        angle_raw= self.encoder_steps_to_angle(max(motor1_steps, motor2_steps))
        angle_deg = self.angle_scale_down(angle_raw)
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

    def dist_scale_down(self, dist_raw):
        dist_actual = 0 
        
        if dist_raw > Constants.FORWARD.value:
            dist_actual = (dist_raw - Constants.FORWARD.value)/Scaling.FORWARD.value
        print(f"dist scaled dowwn = {dist_actual}")
        return dist_actual 

    def dist_scale_up(self, dist_actual):
        dist_raw = dist_actual * Scaling.FORWARD.value + Constants.FORWARD.value
        return dist_raw


    def stop_forward(self):
        self.motor1_in1.off()
        self.motor2_in1.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        dist_raw = self.encoder_steps_to_dist(max(motor1_steps, motor2_steps))
        dist_travelled = self.dist_scale_up(dist_raw)

        """check odometry calcs"""

        # self.update_x(x_diff)
        # self.update_y(y_diff)

        self.update_position(dist_travelled)

        self.reset_encoders()


    def update_position(self, dist_travelled):
        angle_deg = self.th


        angle_rad = math.radians(self.th)
        if 0 <= angle_deg < 90:
            x_diff = math.cos(angle_rad)*dist_travelled
            y_diff = math.sin(angle_rad)*dist_travelled

            # print("1")
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
            self.update_y(0-y_diff)  

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
        self.motor1_in1.off()
        self.motor1_in2.on()
        self.motor2_in1.off()
        self.motor2_in2.on()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed
        
        """check odometry calcs"""

    def stop_backwards(self):
        self.motor1_pwm.off()
        self.motor2_pwm.off()

        self.motor1_in2.off()
        self.motor2_in2.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        dist_raw = self.encoder_steps_to_dist(max(motor1_steps, motor2_steps))
        dist_travelled = self.dist_scale_down(dist_raw)

        self.update_position(dist_travelled)


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


    def dist_input_output_scaling(self, dist_input):
        dist_output = dist_input*Scaling.FORWARD.value + Constants.FORWARD.value
        return dist_output
    
    def dist_output_input_scaling(self, dist_input):
        dist_output = (dist_input -Constants.FORWARD.value)/ Scaling.FORWARD.value
        return dist_output


    def move_forward(self, distance, speed=0.5):
        """
        Moves the robot forward by a specific distance at the given speed.
        """
        dist_raw = self.dist_scale_down(distance)
        encoder_steps = self.dist_to_encoder_steps(dist_raw)

       
        self.reset_encoders()

        # Kp_forward = 0.01  # Proportional gain for forward adjustment

        # Set motor direction for forward movement
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.on()
        self.motor2_in2.off()

        self.motor1_pwm.value = speed*self.left_motor_scale
        self.motor2_pwm.value = speed

        # Start moving forward
        while abs(self.motor1_encoder.steps) < encoder_steps and abs(self.motor2_encoder.steps) < encoder_steps:

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

        self.update_position(dist_travelled)

        self.reset_encoders()

        # print("done\n")

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
        wheel_rotations = abs(encoder_steps)/(CPR* GEAR_RATIO)
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
        
        
        angle_raw = self.angle_scale_down(angle_degrees)
    
        if angle_degrees < 0:
            self.rotate_clockwise(angle_raw, speed)
        else:
            self.rotate_anticlockwise(angle_raw, speed)
    
    def angle_scale_down(self, angle_raw):
        angle_actual = (angle_raw - Constants.ROTATE.value)/Scaling.ROTATE.value
        return angle_actual 

    def angle_scale_up(self, angle_actual):
        angle_raw = angle_actual * Scaling.ROTATE.value + Constants.ROTATE.value
        return angle_raw


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
       
    
    def calc_rotated_angle(self):
        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        print(f"motor1 steps = {motor1_steps} --- motor2_steps = {motor2_steps}")

        angle_deg= self.encoder_steps_to_angle(max(motor1_steps, motor2_steps))

        return angle_deg

    def rotate_to_face_centre(self):
        

        return None 

    def get_dist_moved(self):
        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps
        steps = max(motor1_steps, motor2_steps)
        print(f"STEPSSSSS: {steps}")
        time.sleep(0.1)

        dist_raw = self.encoder_steps_to_dist(steps)
        dist_actual = self.dist_scale_up(dist_raw)
        return dist_actual

    def handle_state(self, frame):
        if self.state == State.CLOSE_TO_TARGET: 
            camNum = 2 
        else:
            camNum = 1
        # track ball and line detect  
        vision_x, vision_y = self.vision.track_ball(frame, camNum)

        print(f"vision_x = {vision_x}  --- vision_y = {vision_y}")

        old_state = self.state

        print(f"(x, y, th) = ({self.x},  {self.y},  {self.th})")
        match self.state:
            
            case State.START_LEFT: # start on the bottom left side of the quadrant (facing 3 o'clock)
                match vision_x:
                    case -1: # no ball detected in frame 
                        self.start_rotating_clockwise(speed=Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATE_LEFT  # rotate on the spot
                    case 2: #   # ball detected on left side of frame
                        self.start_rotating_clockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET
                    case 1: # ball in frame in centre 
                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: 
                            self.start_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # ball in frame to right side 
                        self.start_rotating_clockwise(Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATE_RIGHT_TARGET 

            case State.START_RIGHT: # start on the bottom right side of the quadrant (facing 9 o'clock)
                match vision_x:
                    case -1: # no ball detected in frame 
                        self.start_rotating_anticlockwise(speed=Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATE_EXPLORE  # rotate on the spot
                    case 2: #   # ball detected on left side of frame
                        self.start_rotating_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET
                    case 1: # ball in frame in centre 
                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: 
                            self.start_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # ball in frame to right side 
                        self.start_rotating_clockwise(Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATE_RIGHT_TARGET
            
            case State.ROTATE_EXPLORE: # spinning in place to find a target if it exists 
                match vision_x:
                    case -1: # nothing detected
                        # check if already spun 360 
                        angle_raw = self.calc_rotated_angle() 
                        angle_rotated = self.angle_scale_up(angle_raw)

                        print(f"angle_rotated = {angle_rotated}")

                        if angle_rotated >= 90:
                            self.stop_rotating_anticlockwise()
                            self.start_rotating_clockwise(Speeds.ROTATING_EXPLORE.value)
                            self.state = State.ROTATE_FACE_CENTRE
                        else:
                            # keep spinning 
                            pass
                    case 2: # left  
                        # keep spinning 
                        self.stop_rotating_anticlockwise()
                        self.start_rotating_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET 
                    case 1:
                        self.stop_rotating_anticlockwise() # stop rotating

                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else:
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # to the right 
                        self.stop_rotating_anticlockwise()
                        self.start_rotating_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET 

            case State.ROTATE_EXPLORE2: # spinning in place to find a target if it exists 
                match vision_x:
                    case -1: # nothing detected
                        # check if already spun 360 
        
                        angle_raw = self.calc_rotated_angle() 
                        angle_rotated = self.angle_scale_up(angle_raw)

                        print(f"angle_rotated = {angle_rotated}")

                        if angle_rotated >= 180:
                            self.stop_rotating_anticlockwise()
                           
                            self.state= State.ROTATE_FACE_CENTRE
                            # self.rotate_to_face_centre()

                    case 2: # left 
                        # keep spinning 
                        self.stop_rotating_anticlockwise()
                        self.start_rotating_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET 
                    case 1:
                        self.stop_rotating_anticlockwise() # stop rotating

                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else:
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # to the right 
                        self.stop_rotating_anticlockwise()
                        self.start_rotating_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET 

        
            case State.ROTATE_FACE_CENTRE:
                match vision_x:
                    case -1: # nothing detected, keep spinning
                        angle_raw = self.calc_rotated_angle() 
                        angle_rotated = self.angle_scale_up(angle_raw)

                        print(f"angle_rotated = {angle_rotated}")

                        if angle_rotated >= 45: # stop rotating 
                            self.stop_rotating_anticlockwise()
                            print(f"angle_rotated = {angle_rotated}")
                            self.start_forward(Speeds.MOVING_TO_CENTRE.value)
                            self.state = State.MOVE_TO_CENTRE
                        else:
                            pass # keep rotating
                
                    case 2: # left 
                        # keep spinning     
                        self.stop_rotating_clockwise()
                        self.start_rotating_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET 
                    case 1:
                        self.stop_rotating_clockwise() # stop rotating

                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else:
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # to the right 
                        self.stop_rotating_clockwise()
                        self.start_rotating_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET 
                
            case State.MOVE_TO_CENTRE:
                match vision_x:
                    case -1: # nothing detected, keep moving to centre
                        distance_travelled =  self.get_dist_moved()

                        if distance_travelled >= self.calibrated_centre_move: 
                            self.stop_forward()
                            self.start_rotating_anticlockwise(speed=Speeds.ROTATING_EXPLORE.value)
                            self.state = State.ROTATE_EXPLORE2
                
                    case 2: # left 
                        # keep spinning     
                        self.stop_forward()
                        self.start_rotating_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET 
                    case 1:
                        self.stop_forward()
                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else:
                            self.start_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # to the right 
                        self.stop_forward()
                        self.start_rotating_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET 


            case State.ROTATE_RIGHT_TARGET:
                match vision_x:
                    case -1: # the ball has left the frame 
                        self.stop_rotating_clockwise()
                        self.start_rotating_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_EXPLORE
                    case 2: 
                        self.stop_rotating_clockwise()
                        self.start_rotating_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET
                    case 1: # ball in centre of frame so start moving forward
                        self.stop_rotating_anticlockwise()
                        if vision_y == 1: # close 
                            self.move_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: # not close 
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3:  # ball to right of the frame, keep rotating clockwise     
                        pass

            case State.ROTATE_EXPLORE_FULL_PRIMARY:
                match vision_x:
                    case -1: # nothing detected
                        # check if already spun 360 
                        angle_rotated = self.calc_rotated_angle() 

                        # TODO - fix this into two?
                        if angle_rotated >= 360:
                            self.stop_rotating_anticlockwise()
                            self.start_rotating_anticlockwise(speed=Speeds.ROTATING_EXPLORE.value)
                            self.state = State.ROTATE_EXPLORE2
                            # self.rotate_to_face_centre()

                    case 2: # left 
                        # keep spinning 
                        self.stop_rotating_anticlockwise()
                        self.start_rotating_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET 
                    case 1:
                        self.stop_rotating_anticlockwise() # stop rotating

                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else:
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # to the right 
                        self.stop_rotating_anticlockwise()
                        self.start_rotating_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET 

            case State.ROTATE_LEFT_TARGET:
                match vision_x:
                    case -1: # the ball has left the frame 
                        self.stop_rotating_anticlockwise()
                        self.start_rotating_anticlockwise(Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATE_EXPLORE
                    case 2:  # ball still to left, keep rotating anticlock
                        print("in rotate left, keep in rotate left")
                        pass 
                    case 1: # ball in centre of frame so start moving forward
                        self.stop_rotating_anticlockwise()
                        if vision_y == 1: # close 
                            self.move_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: # not close 
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3:  # ball now to right of the frame, rotate to right  
                        self.stop_rotating_anticlockwise()
                        self.start_rotating_clockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET

            case State.MOVE_TO_TARGET:
                match vision_x:
                    case -1: # the ball has left the frame 
                        self.stop_forward()
                        self.start_rotating_anticlockwise(speed = Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATE_EXPLORE
                    case 2:  # ball to the left so so readjust 
                        self.stop_forward()
                        self.start_rotating_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET
                    case 1: # ball in centre of frame
                        if vision_y == 1: # close 
                            self.stop_forward() #stop moving first
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: # not close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                    case 3:  # ball to right of the frame, stop moving to readjust    
                        self.stop_forward()
                        self.start_rotating_clockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET

            case State.CLOSE_TO_TARGET:
                calc_dist_moved = self.get_dist_moved()
                print(f"dist_moved = {calc_dist_moved}")

                distance_moved = calc_dist_moved 

                # dist_to_move = self.calibrated_close_dist +  self.close_to_target_calibration
                dist_to_move = 55/100 
                print(f"dist_to_move = {dist_to_move}")

                if distance_moved >= dist_to_move:  # blind-spot distance +  bottom frame dist=  0.3 + 0.15
                    self.stop_forward()
                    print("touched ball!")
                    self.start_backward(speed=Speeds.RETREAT_BACK.value)
                    self.state = State.START_RETURN
                else:
                    pass
                
            case State.START_RETURN:
                distance_moved = self.get_dist_moved()
                
                if distance_moved >= self.calibrated_retreat_dist:
                    self.stop_backwards()
                    self.start_rotating_anticlockwise(speed=Speeds.RETURN_ROTATE.value)
                    self.angle_to_rotate_to 
                    self.state = State.ROTATE_TO_FACE_START

            case State.START_RETURN: 
                distance_moved = self.get_dist_moved()
                
                if distance_moved >= self.calibrated_retreat_dist:
                    self.stop_backwards()
                    self.start_rotating_anticlockwise(speed=Speeds.RETURN_ROTATE.value)
                    self.angle_to_rotate_to 
                    self.state = State.ROTATE_TO_FACE_START
                
            case State.ROTATE_TO_FACE_START:
                th_current = self.th
                x_current = self.x
                y_current = self.y

                # taken from vishGPT
                match self.start_pos:
                    case StartPosition.LEFT:
                        theta_rad = math.atan2(y_current,x_current)
                        theta_deg = math.degrees(theta_rad)
                
                    case StartPosition.RIGHT:
                        theta_rad = math.atan2(y_current,-x_current)
                        theta_deg = math.degrees(theta_rad)

                ang_diff = (theta_deg - th_current) % 360
                if ang_diff > 180: # turn anticlockwise 
                    self.start_rotating_anticlockwise(Speeds.RETURN_ROTATE.value)
                    self.angle_to_rotate_to = 360 - ang_diff
                    self.rotate_direction = RotateDirection.ANTICLOCKWISE

                else: # turn clockwise 
                    self.start_rotating_clockwise(Speeds.RETURN_ROTATE.value)
                    self.angle_to_rotate_to = ang_diff
                    self.rotate_direction = RotateDirection.CLOCKWISE
                self.state = State.ROTATING_TO_FACE_START

            case State.ROTATING_TO_FACE_START:
                angle_rotated = self.calc_rotated_angle() 

                if angle_rotated >= self.angle_to_rotate_to:
                    if self.rotate_direction is RotateDirection.ANTICLOCKWISE:
                        self.stop_rotating_anticlockwise()
                    else:
                        self.stop_rotating_anticlockwise()
                    
                    self.start_forward(speed = Speeds.MOVING_TO_START)
                    self.state = State.MOVE_TO_START
        
            case State.MOVE_TO_START: 
                
                # check if you've moved to start position (or close to)
                x_current = self.x
                y_current = self.y

                if x_current <= 0.2 and y_current <= 0.2: 
                    self.stop_forward()
                    self.state = State 

        print(f"old state = {old_state.name} --- new-state = {self.state.name}")














                

            
           
        

    


    

        
  