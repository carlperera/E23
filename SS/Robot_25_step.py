import time
import gpiozero
import math
from State import State
from Vision import Vision
from State import StartPosition
from Servo import Flap, Claw
import RPi.GPIO as GPIO
from enum import Enum

from Vision import VISION_X, VISION_Y

WHEEL_SEP = 238/1000    # previously 227/1000  
WHEEL_RAD = 99.88/1000    #  preciously 61/2/1000 
CPR = 48
GEAR_RATIO = 74.38

MOTOR2_SCALING = 0.75

class Scaling(Enum):
    ROTATE_ANTICLOCKWISE = 1.95
    ROTATE_CLOCKWISE = 2

    ROTATE = 4.2  # 3.7
    FORWARD = 3.486  # 4.2
    BACK = 1.0

class Constants(Enum): 
    ROTATE_ANTICLOCKWISE = 14.8 / 100 + 1500/100
    ROTATE_CLOCKWISE = 21.9 / 100 #  + 1/100

    ROTATE = 3 / 100
    FORWARD = 4.282 / 100
    TRAY_BALL_THRESHOLD = 4
    SIMULATION_DURATION_S = 10*60

class Speeds(Enum):
    MOVING_TO_TARGET = 0.5
    ROTATING_EXPLORE = 0.5
    CLOSE_TO_TARGET = 0.3
    ROTATE_TO_TARGET = 0.5
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
    NOT_ROTATING = -1
    ANTICLOCKWISE = 0 
    CLOCKWISE = 1 


class SERVO_PINS(Enum):
    PIN_FLAP_PWM = 12
    PIN_CLAW_PWM = 13

"""
Dimension of standard singles court (not including the left and right outer edges) is 
width = 8.27 m
height = 23.77 m

Since the given dimensions seeem to have a little bit taken off the height, we have taken this into consideration

"""
# TODO: someone go and measure or ask the dems for the quadrant dimensions 
class COURT_DIMS(Enum):
    COURT_X = 10
    COURT_Y = 4

class COLLECTION_BOX_DIMS(Enum):
    LENGTH = 60/100
    WIDTH = 45/100
    HEIGHT = 16/100
    HEIGHT_TOLERANCE = 2/100



class Robot:    

    def __init__(self, vision: Vision | None, start_pos: StartPosition) -> None:

        # ------- set mode -------
        GPIO.setmode(GPIO.BCM)

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
         # ----------------  Start Position ----------------
        self.start_pos = start_pos
        if start_pos is StartPosition.LEFT:
            self.state = State.START_LEFT
        else:
            self.state = State.START_RIGHT
        
        self.reset_position(0.0,0.0,90)  # set your position (always starting at 90 degrees)
        self.reset_encoders()

        self.close_to_target_calibration = 0.35
        # ----------------  VISION ----------------
        self.vision = vision

        self.angle_to_rotate_to = None
        
        
        # ----------------  BALLS - TRAY ----------------
        self.tray_ball_threshold = Constants.TRAY_BALL_THRESHOLD # number of balls in robot tray at which point robot goes to collection box

        # ----------------  ----------------
        # self.flap = Flap()
        # self.claw = Claw()
        
  
    def shutdown(self):
        self.vision.close()

    def reset_position(self, x, y, th):
        self.x = x
        self.y = y
        self.th = th

    def reset_encoders(self):
        self.motor1_encoder.steps = 0
        self.motor2_encoder.steps = 0

    def update_and_get_orientation(self):
        rotating_direction = self.get_rotating_direction()
        angle_rotated = self.calc_rotated_angle(rotating_direction)
        

        # reset the encoders 
        self.reset_encoders() 

        try:
            if rotating_direction is None:
                raise RuntimeError("ERROR: no rotating direction")
            print(f"rotating direction: {rotating_direction.name}")
        except RuntimeError as e:
            print(f"Error: {e}")

        match(rotating_direction):
            case RotateDirection.ANTICLOCKWISE:
                self.update_orientation(angle_rotated)
            case RotateDirection.CLOCKWISE:
                self.update_orientation(0-angle_rotated)
        return self.get_orientation()
    
    def get_rotating_direction(self):
        # check if rotating clockwise
        if self.motor1_in1.value == 1 and self.motor1_in2.value == 0 and self.motor2_in1.value == 0 and self.motor2_in2.value == 1:
            return RotateDirection.CLOCKWISE
        
        # check if rotating anticlockwise 
        elif self.motor1_in1.value == 0 and self.motor1_in2.value == 1 and self.motor2_in1.value == 1 and self.motor2_in2.value == 0:
            return RotateDirection.ANTICLOCKWISE
        print(f"{self.motor1_in1.value} - {self.motor1_in2.value} - {self.motor2_in1.value} - {self.motor2_in2.value}")
        return None
    
    def get_orientation(self):
        return self.th
    
    def check_rotated_90_at_start(self):
        start_pos = self.start_pos 
        curr_orientation = self.update_and_get_orientation()
        if start_pos == StartPosition.RIGHT: # start RIGHT 
            return curr_orientation >= 180
        else:  # start LEFT # TODO: check if this logic is right -> ask VishGPT if this is goood
            return curr_orientation == 0 or curr_orientation >= 270 # just need to check that it's gone below 0 and then "gone over"

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def distance_per_step(self):
        wheel_rotatations_per_step = 1 / (CPR*GEAR_RATIO)
        return 2*math.pi*self.wheel_radius*wheel_rotatations_per_step


    def turn_off_all(self):
        self.motor1_pwm.off()
        self.motor1_in1.off()
        self.motor1_in2.off()

        self.motor2_pwm.off()
        self.motor1_in1.off()
        self.motor1_in2.off()

    """MISSION METHODS"""
    def start_clockwise(self, speed):
        self.reset_encoders()
        
        # Set motor directions for clockwise rotation
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.off()
        self.motor2_in2.on()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

    def start_anticlockwise(self, speed):
        self.reset_encoders()
        
        # Set motor directions for clockwise rotation
        self.motor1_in1.off()
        self.motor1_in2.on()
        self.motor2_in1.on()
        self.motor2_in2.off()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

    def update_orientation(self, angle_deg):
        self.th = (self.th + angle_deg) % 360

    def update_x(self, x_diff):
        self.x = self.x + x_diff
    
    def update_y(self, y_diff):
        self.y = self.y + y_diff

    def stop_clockwise(self):
        
        self.motor1_pwm.off()
        self.motor2_pwm.off()

        self.motor1_in1.off()
        self.motor1_in2.off()
        self.motor2_in1.off()
        self.motor2_in2.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        angle_raw= self.encoder_steps_to_angle(max(motor1_steps, motor2_steps))
        angle_deg = self.convert_actual_angle_to_encoder_angle(angle_raw)
        print(f"angle_diff = {angle_deg}")
        self.update_orientation(0-angle_deg)

        self.reset_encoders()
    
    def stop_anticlockwise(self):

        self.motor1_pwm.off()
        self.motor2_pwm.off()

        self.motor1_in1.off()
        self.motor1_in2.off()
        self.motor2_in1.off()
        self.motor2_in2.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        angle_raw= self.encoder_steps_to_angle(max(motor1_steps, motor2_steps))
        angle_deg = self.convert_actual_angle_to_encoder_angle(angle_raw)
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

    """ Converting between encoder <-> actual distance
    -> the encoder thinks it has moved more than it actually has
    """
    def convert_encoder_dist_to_actual(self, dist_encoder):
        """
        Converts the distance the encoder thinks it has moved to 
        the distance that has been actually moved 
        """
        dist_actual = dist_encoder * Scaling.FORWARD.value + Constants.FORWARD.value
        return dist_actual

    def convert_actual_to_encoder_dist(self, dist_actual):
        """
        Converts the distance actually needed to be moved to the distance the encoder thinks it needs to move 
        """
        dist_encoder = 0 
        
        if dist_actual > Constants.FORWARD.value:
            dist_encoder = (dist_actual - Constants.FORWARD.value)/Scaling.FORWARD.value
        return dist_encoder 

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

        # x is pos left to right 
        start_pos = self.start_pos
        
        if 0 <= angle_deg < 90:
            print("1")
            x_diff = math.cos(angle_rad)*dist_travelled
            y_diff = math.sin(angle_rad)*dist_travelled

            if start_pos:  # right side 
                self.update_x(0-x_diff)
            else:          # left side 
                self.update_x(x_diff)

            self.update_y(y_diff)   

        elif 90 <= angle_deg < 180:
            angle = 180-angle_deg
            print("2")
            x_diff = math.cos(angle_rad)*dist_travelled
            y_diff = math.sin(math.radians(angle))*dist_travelled

            if start_pos: # right side 
                self.update_x(x_diff)
            else:         # left side 
                self.update_x(0-x_diff)

            self.update_y(y_diff)  

        elif 180 <= angle_deg < 270:
            angle = 270-angle_deg

            print("3")
            x_diff = math.sin(math.radians(angle))*dist_travelled
            y_diff = math.cos(math.radians(angle))*dist_travelled 

            if start_pos:
                self.update_x(x_diff)
            else:
                self.update_x(0-x_diff)
            self.update_y(0-y_diff)  

        else:
            angle = 360-angle_deg

            print("4")
            x_diff = math.cos(math.radians(angle))*dist_travelled
            y_diff = math.sin(math.radians(angle))*dist_travelled 
            if start_pos:
                self.update_x(0-x_diff)
            else:
                self.update_x(x_diff)
            self.update_y(0-y_diff)  

        print(f"new position ->(x, y, th) = ({self.x},  {self.y},  {self.th})")
        

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

    def move_test(self, distance, speed):

        if distance > 0: 
            self.forward_test(distance, speed)
        else:
            self.backward_test(abs(distance), speed)

    
    """ START: ------------------------------- CALIBRATION -------------------------------
    """
    def move_calibrate(self, distance, speed):

        if distance > 0: 
            self.forward_calibrate(distance, speed)
        else:
            self.backward_calibrate(abs(distance), speed)

    def backward_calibrate(self, distance, speed):
        encoder_steps = self.dist_to_encoder_steps(distance)

        self.reset_encoders()

        # Set motor direction for forward movement
        self.motor1_in1.off()
        self.motor1_in2.on()
        self.motor2_in1.off()
        self.motor2_in2.on()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        # Start moving forward
        while abs(self.motor1_encoder.steps) < encoder_steps and abs(self.motor2_encoder.steps) < encoder_steps:
            time.sleep(0.01)

        self.motor1_pwm.off()
        self.motor2_pwm.off()

        motor1_steps = abs(self.motor1_encoder.steps)
        motor2_steps = abs(self.motor2_encoder.steps)

        dist_travelled = self.encoder_steps_to_dist(max(motor1_steps, motor2_steps))

        self.update_position(dist_travelled)
        self.reset_encoders()


    def forward_calibrate(self, distance, speed):
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
        while abs(self.motor1_encoder.steps) < encoder_steps and abs(self.motor2_encoder.steps) < encoder_steps:
            time.sleep(0.01)

        self.motor1_pwm.off()
        self.motor2_pwm.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        dist_travelled = self.encoder_steps_to_dist(max(motor1_steps, motor2_steps))

        self.update_position(dist_travelled)
        self.reset_encoders()

    def rotate_calibrate(self, angle_deg, speed):
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
        self.reset_encoders()
        
        # Determine direction based on the sign of the angle
    
        if angle_deg < 0:
            self.rotate_clockwise_calibrate(abs(angle_deg), speed)
        else:
            self.rotate_anticlockwise_calibrate(angle_deg, speed)


    
    def rotate_clockwise_calibrate(self, angle, speed=0.5):
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

    
    def rotate_anticlockwise_calibrate(self, angle, speed=0.5):
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

    """ END: ------------------------------- CALIBRATION -------------------------------
    """
    


    def backward_test(self, distance, speed):
        encoder_steps = self.dist_to_encoder_steps(distance)
        self.reset_encoders()
        # Set motor direction for backward movement
        self.motor1_in1.off()
        self.motor1_in2.on()
        self.motor2_in1.off()
        self.motor2_in2.on()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        # Start moving backward
        while abs(self.motor1_encoder.steps) < encoder_steps or abs(self.motor2_encoder.steps) < encoder_steps:
            time.sleep(0.01)  # Small delay for sensor feedback

        # Stop motors
        self.motor1_pwm.off()
        self.motor2_pwm.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        dist_travelled = self.encoder_steps_to_dist(max(motor1_steps, motor2_steps))

        self.update_position(dist_travelled)
        self.reset_encoders()


    def forward_test(self, distance, speed):
        dist_encoders = self.convert_actual_to_encoder_dist(distance)
        encoder_steps = self.dist_to_encoder_steps(dist_encoders)

        self.reset_encoders()

        # Set motor direction for forward movement
        self.motor1_in1.on()
        self.motor1_in2.off()
        self.motor2_in1.on()
        self.motor2_in2.off()

        self.motor1_pwm.value = speed
        self.motor2_pwm.value = speed

        # Start moving forward
        while abs(self.motor1_encoder.steps) < encoder_steps and abs(self.motor2_encoder.steps) < encoder_steps:
            time.sleep(0.01)

        self.motor1_pwm.off()
        self.motor2_pwm.off()

        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        print(f"motor1_steps: {motor1_steps}  --- motor2_steps: {motor2_steps}")

        dist_travelled = self.encoder_steps_to_dist(max(motor1_steps, motor2_steps))

        self.update_position(dist_travelled)
        self.reset_encoders()

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
        self.reset_encoders()
        
        # Determine direction based on the sign of the angle
        # angle_raw = self.convert_actual_angle_to_encoder_angle(angle_degrees)  # scaled down
    
        if angle_degrees < 0:
            angle_raw = self.convert_encoder_to_actual_angle_clockwise(abs(angle_degrees))
            self.rotate_clockwise(angle_raw, speed)
        else:
            angle_raw = self.convert_encoder_to_actual_angle_anticlockwise(angle_degrees)
            self.rotate_anticlockwise(angle_raw, speed)
    
    def convert_actual_angle_to_encoder_angle(self, angle_raw):
        """ scales the user desired input angle into the encoder angle (right now it is SCALED DOWN)
        """
        angle_actual = (angle_raw - Constants.ROTATE.value)/Scaling.ROTATE.value
        return angle_actual 

    def convert_encoder_angle_to_actual_angle(self, angle_actual, rotating_direction):
        """ scales raw encoder angle into the user desired input (right now it is SCALED UP)
        """
        match rotating_direction:
            case RotateDirection.ANTICLOCKWISE:
                return self.convert_encoder_to_actual_angle_anticlockwise(angle_actual)
            case RotateDirection.CLOCKWISE:
                return self.convert_encoder_to_actual_angle_clockwise(angle_actual)

    def convert_encoder_to_actual_angle_clockwise(self, angle_actual):
        if (angle_actual - Constants.ROTATE_CLOCKWISE.value ) <= 0:
            angle_raw = 0 
            return angle_raw

        angle_raw = (angle_actual - Constants.ROTATE_CLOCKWISE.value )/Scaling.ROTATE_CLOCKWISE.value 
        print(angle_raw)
        return angle_raw
    
    def convert_encoder_to_actual_angle_anticlockwise(self, angle_actual):
        angle_raw = (angle_actual - Constants.ROTATE_ANTICLOCKWISE.value)/Scaling.ROTATE_ANTICLOCKWISE.value
        print(angle_raw) 
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

        while abs(self.motor1_encoder.steps) < encoder_steps and abs(self.motor2_encoder.steps) < encoder_steps:
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

        while abs(self.motor1_encoder.steps) < encoder_steps and abs(self.motor2_encoder.steps) < encoder_steps:
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
       

    def calc_rotated_angle(self, rotating_direction):


        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps

        print(f"motor1 steps = {motor1_steps} --- motor2_steps = {motor2_steps}")
        angle_deg = self.encoder_steps_to_angle(max(motor1_steps, motor2_steps))
        angle_rotated = self.convert_encoder_angle_to_actual_angle(angle_deg, rotating_direction)

        return angle_rotated
    
    def check_rotated_by_angle(self, angle):
        return self.calc_rotated_angle() >= angle

    def rotate_to_face_centre(self):
        return None 

    def get_dist_moved(self):
        motor1_steps = self.motor1_encoder.steps
        motor2_steps = self.motor2_encoder.steps
        steps = max(motor1_steps, motor2_steps)
        # print(f"STEPSSSSS: {steps}")
        time.sleep(0.1)

        dist_raw = self.encoder_steps_to_dist(steps)
        dist_actual = self.dist_scale_up(dist_raw)
        return dist_actual

    def determine_face_centre_angle_dir(self):
        """
        Figure out how much to turn by to face the centre, and also which direction (0 - clockwise, 1 - anticlock)
        """

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

        angle_diff = (theta_deg - th_current) % 360
        return angle_diff
    

    def handle_state(self, frame):
        # TODO: add all of the secondary states here 
        SECONDARY_CAM_STATES = [State.CLOSE_TO_TARGET, State.MOVE_TO_TARGET_SECONDARY, State.ROTATE_RIGHT_SECONDARY, State.ROTATE_LEFT_SECONDARY]

        # TODO: changing the camNums is not just related to a single state anymore 
        if self.state in SECONDARY_CAM_STATES:
            camNum = 2 
        else:
            camNum = 1
        
        # track ball and line detect  
        vision_x, vision_y = self.vision.track_ball(frame, camNum)
        
        print(f"vision_x = {vision_x}  --- vision_y = {vision_y} --- camNum = {camNum}")

        old_state = self.state

        print(f"(x, y, th) = ({self.x},  {self.y},  {self.th})")

        match self.state:

            # -------------------------  START SEQUENCE  -------------------------
            case State.START_LEFT: # start on the bottom left side of the quadrant (facing 3 o'clock)
                match vision_x:
                    case -1: # no ball detected in frame 
                        self.start_clockwise(speed=Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATING_START_LEFT_EXPLORE  # rotate on the spot (anticlock)
                    case 2: #   # ball detected on left side of frame # TODO: this is a realistic state? 
                        self.start_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET
                    case 1: # ball in frame in centre 
                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET # switch to secondary camera
                        else: # not close 
                            self.start_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # ball in frame to right side 
                        self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET 

            case State.START_RIGHT: # start on the bottom right side of the quadrant (facing 9 o'clock)
                match vision_x:
                    case -1: # no ball detected in frame 
                        self.start_anticlockwise(speed=Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATING_START_RIGHT_EXPLORE  # rotate on the spot
                    case 2: #   # ball detected on left side of frame
                        self.start_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET
                    case 1: # ball in frame in centre 
                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET 
                        else: # not close 
                            self.start_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # ball in frame to right side  # TODO: is this realistic state?
                        self.start_clockwise(Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATE_RIGHT_TARGET

            case State.ROTATING_START_LEFT_EXPLORE:
                match vision_x:
                    case -1: # no ball detected in frame 
                        # check if rotated to 90 degrees already 
                        if self.check_rotated_90_at_start():
                            self.stop_clockwise()
                            self.start_anticlockwise(speed=Speeds.ROTATING_EXPLORE.value) # rotate to face the centre
                            self.state = State.ROTATING_FACE_CENTRE_START_LEFT # TODO: could merge this with the ROTATE_FACE_CENTRE_BALL state?
                        else:
                            pass # keep rotating  
                    case 2: #   # ball detected on left side of frame
                        self.stop_clockwise()
                        self.start_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET
                    case 1: # ball in frame in centre 
                        self.stop_clockwise()
                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: 
                            self.start_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # ball in frame to right side 
                        self.start_clockwise(Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATE_RIGHT_TARGET

            case State.ROTATING_START_RIGHT_EXPLORE:
                match vision_x:
                    case -1: # no ball detected in frame 
                        # check if rotated to 90 degrees already 
                        if self.check_rotated_90_at_start():
                            self.stop_anticlockwise()
                            self.start_clockwise(speed=Speeds.ROTATE_TO_CENTRE.value) # rotate to face the centre
                            self.state = State.ROTATING_FACE_CENTRE_START_RIGHT # TODO: could merge this with the ROTATE_FACE_CENTRE_BALL state?
                        else:
                            pass # keep rotating  
                    case 2: #   # ball detected on left side of frame
                        self.stop_anticlockwise()
                        self.start_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET
                    case 1: # ball in frame in centre 
                        self.stop_anticlockwise()
                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: 
                            self.start_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # ball in frame to right side 
                        self.stop_anticlockwise()
                        self.start_clockwise(Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATE_RIGHT_TARGET

            case State.ROTATING_FACE_CENTRE_START_LEFT:

                if self.update_and_get_orientation() >= 45: 
                    self.stop_anticlockwise()
                    self.start_forward(speed=Speeds.MOVING_TO_CENTRE.value)
                    self.state = State.MOVE_TO_CENTRE_BALL
                else:
                    pass # keep rotating 

            case State.ROTATING_FACE_CENTRE_START_RIGHT:
                if self.update_and_get_orientation() <= 135:
                    self.stop_clockwise()
                    self.start_forward(speed=Speeds.MOVING_TO_CENTRE.value)
                    self.state = State.MOVE_TO_CENTRE_BALL
                else:
                    pass # keep rotating
            

            # -------------------------  EXPLORE BALL  -------------------------
            # rotate full 360 (Twice)
            case State.ROTATE_EXPLORE_FULL_PRIMARY_PART_1:
                match vision_x:
                        case -1: # nothing detected
                            if self.check_rotated_by_angle(180):  # check if already spun 360 
                                self.stop_anticlockwise()

                                # second part of the full 360
                                self.start_anticlockwise()
                                self.state = State.ROTATE_EXPLORE_FULL_PRIMARY_PART_2
                        case 2: # left (keep spinning anticlockwise)
                            self.stop_anticlockwise()
                            # start rotating again
                            self.start_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                            self.state = State.ROTATE_LEFT_TARGET 
                        case 1:
                            self.stop_anticlockwise() # stop rotating
                            
                            if vision_y == 1: # close 
                                self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                                self.state = State.CLOSE_TO_TARGET
                            else: # not close 
                                self.start_forward(Speeds.MOVING_TO_TARGET.value)
                                self.state = State.MOVE_TO_TARGET
                        case 3: # to the right 
                            self.stop_anticlockwise()
                            self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                            self.state = State.ROTATE_RIGHT_TARGET 

            case State.ROTATE_EXPLORE_FULL_PRIMARY_PART_2:
                match vision_x:
                    case -1: # nothing detected
                        # check if already spun 360 
                        if self.check_rotated_by_angle(180):
                            self.stop_anticlockwise()

                            # since no ball spotted, rotate to face the centre and move to it(maximise vision)
                            angle_diff = self.determine_face_centre_angle_dir()

                            if angle_diff > 180: # turn anticlockwise 
                                self.start_anticlockwise(Speeds.RETURN_ROTATE.value)
                                self.angle_to_rotate_to = 360 - angle_diff
                                self.state= State.ROTATE_FACE_CENTRE_BALL

                            else: # turn clockwise 
                                self.start_clockwise(Speeds.RETURN_ROTATE.value)
                                self.angle_to_rotate_to = angle_diff
                                self.state = State.ROTATE_FACE_CENTRE_BALL

                    case 2: # left 
                        # keep spinning 
                        self.stop_anticlockwise()
                        self.start_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET 
                    case 1:
                        self.stop_anticlockwise() # stop rotating

                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else:
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3: # to the right 
                        self.stop_anticlockwise()
                        self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET 
            
            case State.ROTATE_FACE_CENTRE_BALL:
                rotating_direction = self.get_rotating_direction()
            
                match vision_x:
                    case -1: # no ball detected 
                        angle_to_rotate_to = self.angle_to_rotate_to
                        curr_orientation = self.update_and_get_orientation()

                        match rotating_direction:
                            case RotateDirection.ANTICLOCKWISE:
                                if curr_orientation >= angle_to_rotate_to:
                                    self.stop_anticlockwise()
                                    self.start_forward(Speeds.MOVING_TO_CENTRE.value)
                                    self.state = State.MOVE_TO_CENTRE_BALL
                                else:
                                    pass # keep rotating anticlockwise

                            case RotateDirection.CLOCKWISE:
                                if curr_orientation <= angle_to_rotate_to:
                                    self.stop_clockwise()
                                    self.start_forward(Speeds.MOVING_TO_CENTRE.value)
                                    self.state = State.MOVE_TO_CENTRE_BALL
                                else:
                                    pass # keep rotating clockwise
                    case 2:
                        match rotating_direction:
                            case RotateDirection.ANTICLOCKWISE:
                                self.stop_anticlockwise()
                                self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                                self.state = State.ROTATE_LEFT_TARGET
                                
                            case RotateDirection.CLOCKWISE:
                                self.stop_clockwise()
                                self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                                self.state = State.ROTATE_LEFT_TARGET
                        
                    case 1:
                        match rotating_direction:
                            case RotateDirection.ANTICLOCKWISE:
                                self.stop_anticlockwise()

                            case RotateDirection.CLOCKWISE:
                                self.stop_clockwise()
                    
                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: #not close
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET

                    case 3:
                        match rotating_direction:
                            case RotateDirection.ANTICLOCKWISE:
                                self.stop_anticlockwise()
                                self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                                self.state = State.ROTATE_RIGHT_TARGET
                                
                            case RotateDirection.CLOCKWISE:
                                self.stop_clockwise()
                                self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                                self.state = State.ROTATE_RIGHT_TARGET

            case State.MOVE_TO_CENTRE_BALL:
                match vision_x:
                    case -1: # nothing detected, keep moving to centre
                        # check we've already reached the centre
                        distance_travelled =  self.get_dist_moved()

                        if distance_travelled >= self.calibrated_centre_move:  # TODO: replace with odometry?
                            self.stop_forward()
                            self.start_anticlockwise(speed=Speeds.ROTATING_EXPLORE.value)
                            self.state = State.ROTATING_START_RIGHT_EXPLORE
                
                    case 2: # left 
                        # keep spinning     
                        self.stop_forward()
                        self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
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
                        self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET 

            # -------------------------  GO TO TARGET  ------------------------- 
            case State.ROTATE_RIGHT_TARGET:
                match vision_x:
                    case -1: # the ball has left the frame 
                        self.stop_clockwise()
                        self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATING_START_RIGHT_EXPLORE
                    case 2: 
                        self.stop_clockwise()
                        self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET
                    case 1: # ball in centre of frame so start moving forward
                        self.stop_anticlockwise()
                        if vision_y == 1: # close 
                            self.move_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: # not close 
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3:  # ball to right of the frame, keep rotating clockwise     
                        pass
            case State.ROTATE_LEFT_TARGET:
            
                match vision_x:
                    case -1: # the ball has left the frame 
                        self.stop_anticlockwise()
                        self.start_anticlockwise(Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATING_START_RIGHT_EXPLORE
                    case 2:  # ball still to left, keep rotating anticlock
                        print("in rotate left, keep in rotate left")
                        pass 
                    case 1: # ball in centre of frame so start moving forward
                        self.stop_anticlockwise()
                        if vision_y == 1: # close 
                            self.move_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: # not close 
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3:  # ball now to right of the frame, rotate to right  
                        self.stop_anticlockwise()
                        self.start_clockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET

            case State.MOVE_TO_TARGET:
                match vision_x:
                    case -1: # nothing detected, keep moving to centre
                        distance_travelled =  self.get_dist_moved()

                        if distance_travelled >= self.calibrated_centre_move: 
                            self.stop_forward()
                            self.start_anticlockwise(speed=Speeds.ROTATING_EXPLORE.value)
                            self.state = State.ROTATE_EXPLORE_FULL_PRIMARY_PART_1
                    case 2:  # ball to the left so so readjust 
                        self.stop_forward()
                        self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
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
                        self.start_clockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET
                
            case State.CLOSE_TO_TARGET:
                # Need secondary camera rotate left and right 
                if vision_x == 4 and vision_y == 4:
                    self.stop_forward()
                    print("GRABBED ball!")
                    # self.start_backward(speed=Speeds.RETREAT_BACK.value)
                    self.state = State.PICKUP_BALL
                else:
                    pass

            # -------------------------  FIND BOX -------------------------
            case State.ROTATE_FACE_CENTRE_BOX:
                rotating_direction = self.get_rotating_direction()
                angle_to_rotate_to = self.angle_to_rotate_to
                curr_orientation = self.update_and_get_orientation()

                match rotating_direction:
                    case RotateDirection.ANTICLOCKWISE:
                        if curr_orientation >= angle_to_rotate_to:
                            self.stop_anticlockwise()
                            self.start_forward(Speeds.MOVING_TO_CENTRE.value)
                            self.state = State.MOVE_TO_CENTRE_BOX
                        else:
                            pass # keep rotating anticlockwise

                    case RotateDirection.CLOCKWISE:
                        if curr_orientation <= angle_to_rotate_to:
                            self.stop_clockwise()
                            self.start_forward(Speeds.MOVING_TO_CENTRE.value)
                            self.state = State.MOVE_TO_CENTRE_BOX
                        else:
                            pass # keep rotating clockwise
        
            case State.MOVE_TO_CENTRE_BOX:
                # check odometry to see if moved to centre 

                match vision_x:
                    case VISION_X.no_box_detected: # nothing detected, keep moving to centre

                        # check we've already reached the centre
                        distance_travelled =  self.get_dist_moved()

                        if distance_travelled >= self.calibrated_centre_move:  # TODO: replace with odometry?
                            self.stop_forward()
                            self.start_anticlockwise(speed=Speeds.ROTATING_EXPLORE.value)
                            self.state = State.ROTATE_EXPLORE_BOX_PART_1

                    case VISION_X.box_left: # left 
                        # keep spinning     
                        self.stop_forward()
                        self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_BOX 

                    case VISION_X.box_centre:
                        self.stop_forward() 
                        # TODO: differentiate when the box is close and not close using vision_y
                        if vision_y == 1: # close to box 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_BOX
                        else:
                            self.start_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case VISION_X.ball_right: # to the right 
                        self.stop_forward()
                        self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET 
            # TODO: replace with rotate 360 once motor step count issue fixed 
            case State.ROTATE_EXPLORE_BOX_PART_1:
                match vision_x:
                        case VISION_X.no_ball_detected: # nothing detected
                            if self.check_rotated_by_angle(180):  # check if already spun 360 
                                self.stop_anticlockwise()

                                # second part of the full 360
                                self.start_anticlockwise()
                                self.state = State.ROTATE_EXPLORE_BOX_PART_2
                        case VISION_X.box_left: # left (keep spinning anticlockwise)
                            self.stop_anticlockwise()
                            # start rotating again
                            self.start_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                            self.state = State.ROTATE_LEFT_BOX 
                        case VISION_X.box_centre:
                            self.stop_anticlockwise() # stop rotating
                            
                            if vision_y == 1: # close 
                                self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                                self.state = State.CLOSE_TO_BOX
                            else: # not close 
                                self.start_forward(Speeds.MOVING_TO_TARGET.value)
                                self.state = State.MOVE_TO_BOX
                        case VISION_X.box_right: # to the right 
                            self.stop_anticlockwise()
                            self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                            self.state = State.ROTATE_RIGHT_BOX 

            case State.ROTATE_EXPLORE_BOX_PART_2:
                match vision_x:
                    case VISION_X.no_ball_detected: # nothing detected
                        # check if already spun 360 
                        if self.check_rotated_by_angle(180):
                            self.stop_anticlockwise()

                            # since no ball spotted, rotate to face the centre and move to it(maximise vision)
                            angle_diff = self.determine_face_centre_angle_dir()

                            if angle_diff > 180: # turn anticlockwise 
                                self.start_anticlockwise(Speeds.RETURN_ROTATE.value)
                                self.angle_to_rotate_to = 360 - angle_diff
                                self.state= State.ROTATE_FACE_CENTRE_BOX

                            else: # turn clockwise 
                                self.start_clockwise(Speeds.RETURN_ROTATE.value)
                                self.angle_to_rotate_to = angle_diff
                                self.state = State.ROTATE_FACE_CENTRE_BOX

                    case VISION_X.box_left: # left 
                        # keep spinning 
                        self.stop_anticlockwise()
                        self.start_anticlockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_TARGET 
                    case VISION_X.box_centre:
                        self.stop_anticlockwise() # stop rotating

                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_BOX
                        else:
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_BOX
                    case VISION_X.box_right: # to the right 
                        self.stop_anticlockwise()
                        self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_TARGET 

            case State.ROTATE_FACE_CENTRE_BOX:
                rotating_direction = self.get_rotating_direction()
            
                match vision_x:
                    case -1: # no ball detected 
                        angle_to_rotate_to = self.angle_to_rotate_to
                        curr_orientation = self.update_and_get_orientation()

                        match rotating_direction:
                            case RotateDirection.ANTICLOCKWISE:
                                if curr_orientation >= angle_to_rotate_to:
                                    self.stop_anticlockwise()
                                    self.start_forward(Speeds.MOVING_TO_CENTRE.value)
                                    self.state = State.MOVE_TO_CENTRE_BALL
                                else:
                                    pass # keep rotating anticlockwise

                            case RotateDirection.CLOCKWISE:
                                if curr_orientation <= angle_to_rotate_to:
                                    self.stop_clockwise()
                                    self.start_forward(Speeds.MOVING_TO_CENTRE.value)
                                    self.state = State.MOVE_TO_CENTRE_BALL
                                else:
                                    pass # keep rotating clockwise
                    case 2:
                        match rotating_direction:
                            case RotateDirection.ANTICLOCKWISE:
                                self.stop_anticlockwise()
                                self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                                self.state = State.ROTATE_LEFT_TARGET
                                
                            case RotateDirection.CLOCKWISE:
                                self.stop_clockwise()
                                self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                                self.state = State.ROTATE_LEFT_TARGET
                        
                    case 1:
                        match rotating_direction:
                            case RotateDirection.ANTICLOCKWISE:
                                self.stop_anticlockwise()

                            case RotateDirection.CLOCKWISE:
                                self.stop_clockwise()
                    
                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_TARGET
                        else: #not close
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_TARGET
                    case 3:
                        match rotating_direction:
                            case RotateDirection.ANTICLOCKWISE:
                                self.stop_anticlockwise()
                                self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                                self.state = State.ROTATE_RIGHT_TARGET
                                
                            case RotateDirection.CLOCKWISE:
                                self.stop_clockwise()
                                self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                                self.state = State.ROTATE_RIGHT_TARGET

            case State.MOVE_TO_CENTRE_BOX:
                match vision_x:
                    case -1: # nothing detected, keep moving to centre
                        # check we've already reached the centre
                        distance_travelled =  self.get_dist_moved()

                        if distance_travelled >= self.calibrated_centre_move:  # TODO: replace with odometry?
                            self.stop_forward()
                            self.start_anticlockwise(speed=Speeds.ROTATING_EXPLORE.value)
                            self.state = State.ROTATE_EXPLORE_BOX_PART_1
                
                    case 2: # left 
                        # keep spinning     
                        self.stop_forward()
                        self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_BOX 
                    case 1:
                        self.stop_forward()
                        if vision_y == 1: # close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_BOX
                        else:
                            self.start_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_BOX
                    case 3: # to the right 
                        self.stop_forward()
                        self.start_clockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_BOX 

            # -------------------------  LOCKED ONTO BOX -------------------------
            case State.ROTATE_LEFT_BOX:
                match vision_x:
                    case -1: # the ball has left the frame 
                        self.stop_anticlockwise()
                        self.start_anticlockwise(Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATING_START_RIGHT_EXPLORE
                    case 2:  # ball still to left, keep rotating anticlock
                        print("in rotate left, keep in rotate left")
                        pass 
                    case 1: # ball in centre of frame so start moving forward
                        self.stop_anticlockwise()
                        if vision_y == 1: # close 
                            self.move_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_BOX
                        else: # not close 
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_BOX
                    case 3:  # ball now to right of the frame, rotate to right  
                        self.stop_anticlockwise()
                        self.start_clockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_BOX 
            case State.MOVE_TO_BOX:
                match vision_x:
                    case -1: # the ball has left the frame 
                        self.stop_forward()
                        self.start_anticlockwise(speed = Speeds.ROTATING_EXPLORE.value)
                        self.state = State.ROTATING_START_RIGHT_EXPLORE
                    case 2:  # ball to the left so so readjust 
                        self.stop_forward()
                        self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_BOX
                    case 1: # ball in centre of frame
                        if vision_y == 1: # close 
                            self.stop_forward() #stop moving first
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_BOX
                        else: # not close 
                            self.start_forward(Speeds.CLOSE_TO_TARGET.value)
                    case 3:  # ball to right of the frame, stop moving to readjust    
                        self.stop_forward()
                        self.start_clockwise(speed=Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_RIGHT_BOX

            case State.ROTATE_RIGHT_BOX:
                match vision_x:
                    case -1: # the ball has left the frame 
                        self.stop_clockwise()
                        self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATING_START_RIGHT_EXPLORE
                    case 2: 
                        self.stop_clockwise()
                        self.start_anticlockwise(Speeds.ROTATE_TO_TARGET.value)
                        self.state = State.ROTATE_LEFT_BOX
                    case 1: # ball in centre of frame so start moving forward
                        self.stop_anticlockwise()
                        if vision_y == 1: # close 
                            self.move_forward(Speeds.CLOSE_TO_TARGET.value)
                            self.state = State.CLOSE_TO_BOX
                        else: # not close 
                            self.move_forward(Speeds.MOVING_TO_TARGET.value)
                            self.state = State.MOVE_TO_BOX
                    case 3:  # ball to right of the frame, keep rotating clockwise     
                        pass
            
            case State.CLOSE_TO_BOX:
                calc_dist_moved = self.get_dist_moved()
                print(f"dist_moved = {calc_dist_moved}")

                distance_moved = calc_dist_moved 

                # dist_to_move = self.calibrated_close_dist +  self.close_to_target_calibration
                dist_to_move = 55/100 
                print(f"dist_to_move = {dist_to_move}")

                if vision_x == 4 and vision_y == 4:
                    self.stop_forward()
                    print("GRABBED ball!")
                    self.start_backward(speed=Speeds.RETREAT_BACK.value)
                    self.state = State.PICKUP_BALL
                else:
                    pass
            
            # case State.TURN_BACK_TO_BOX: # t
                
                

            # -------------------------  DEPOSIT INTO BOX -------------------------
            
            
        
        
            
    

            
        print(f"old state = {old_state.name} --- new-state = {self.state.name}")














                

            
           
        

    


    

        
  
