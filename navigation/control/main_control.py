from  DiffDriveRobot import DiffDriveRobot
from RobotController import RobotController
from TentaclePlanner import TentaclePlanner

import numpy as np
import time
import RPi.GPIO as GPIO
import math
from threading import Thread, Lock
from functools import partial

# ----------- GLOBALS ------------
motor1_prev_A = 0
motor1_prev_B = 0
motor1_counts = 0

motor2_prevA = 0
motor2_prevB = 0
motor2_counts = 0

# ----------- CONSTANTS -------------
CPR = 48
GEAR_RATIO = 74.83

# MOTOR 1 (left)
PIN_MOTOR1_IN1 = 17 # LOW
PIN_MOTOR1_IN2 = 27 # LOW 
PIN_MOTOR1_PWM_ENABLE = 18 # LOW
PIN_MOTOR1_A_OUT = 21# LOW 
PIN_MOTOR1_B_OUT = 20 # LOW 

# MOTOR 2 (right)
PIN_MOTOR2_IN1 = 23 # LOW - good
PIN_MOTOR2_IN2 = 24 # LOW -good 
PIN_MOTOR2_PWM_ENABLE = 9 # LOW - good 
PIN_MOTOR2_A_OUT = 14# LOW - good 
PIN_MOTOR2_B_OUT = 15 # LOW - good

# GPIO - LOW AND HIGH
LOW = GPIO.LOW
HIGH = GPIO.HIGH

WHEEL_SEP = (147 + 64)/1000
WHEEL_RAD = (53/2)/1000

# GPIO.output(PIN_MOTOR1_IN1, GPIO.HIGH)
# GPIO.output(PIN_MOTOR1_IN2, GPIO.LOW)
# GPIO.output(PIN_MOTOR2_IN1, GPIO.HIGH)
# GPIO.output(PIN_MOTOR2_IN2, GPIO.LOW)

# motor1_enable_pwm.start(100) # initial speed
# motor2_enable_pwm.start(100) # initial speed

mutex_motor1_counts = Lock()
mutex_motor2_counts = Lock()

def ISR_motor1(channel) -> None:
    """The frequency of this interrupt service routine is dependent on the duty cycle of 
    A and B output feedbacks 
    
    """
    global motor1_prev_A, motor1_prev_B, motor1_counts

    motor1_A = GPIO.input(PIN_MOTOR1_A_OUT)
    motor1_B = GPIO.input(PIN_MOTOR1_A_OUT)

    if (motor1_prev_A == LOW and motor1_A == HIGH and motor1_prev_B == LOW and motor1_B == LOW) or \
        (motor1_prev_A == HIGH and motor1_A == HIGH and motor1_prev_B == LOW and motor1_B == HIGH) or \
        (motor1_prev_A == HIGH and motor1_A == LOW and motor1_prev_B == HIGH and motor1_B == HIGH) or \
        (motor1_prev_A == LOW and motor1_A == LOW and motor1_prev_B == HIGH and motor1_B == LOW):

        with mutex_motor1_counts:
            motor1_counts += 1  # Clockwise

    elif (motor1_prev_A == LOW and motor1_A == LOW and motor1_prev_B == LOW and motor1_B == HIGH) or \
         (motor1_prev_A == LOW and motor1_A == HIGH and motor1_prev_B == HIGH and motor1_B == HIGH) or \
         (motor1_prev_A == HIGH and motor1_A == HIGH and motor1_prev_B == HIGH and motor1_B == LOW) or \
         (motor1_prev_A == HIGH and motor1_A == LOW and motor1_prev_B == LOW and motor1_B == LOW):
        with mutex_motor1_counts:
            motor1_counts -= 1 #anticlockwise
    

    motor1_prev_A = motor1_A
    motor1_prev_B = motor1_B 

    return None 

def ISR_motor2(channel) -> None:
    """The frequency of this interrupt service routine is dependent on the duty cycle of 
    A and B output feedbacks 
    
    """
    global motor2_prev_A, motor2_prev_B, motor2_counts

    motor2_A = GPIO.input(PIN_MOTOR2_A_OUT)
    motor2_B = GPIO.input(PIN_MOTOR2_A_OUT)

    if (motor2_prev_A == LOW and motor2_A == HIGH and motor2_prev_B == LOW and motor2_B == LOW) or \
        (motor2_prev_A == HIGH and motor2_A == HIGH and motor2_prev_B == LOW and motor2_B == HIGH) or \
        (motor2_prev_A == HIGH and motor2_A == LOW and motor2_prev_B == HIGH and motor2_B == HIGH) or \
        (motor2_prev_A == LOW and motor2_A == LOW and motor2_prev_B == HIGH and motor2_B == LOW):
        with mutex_motor2_counts:
            motor2_counts += 1  # Clockwise

    elif (motor2_prev_A == LOW and motor2_A == LOW and motor2_prev_B == LOW and motor2_B == HIGH) or \
         (motor2_prev_A == LOW and motor2_A == HIGH and motor2_prev_B == HIGH and motor2_B == HIGH) or \
         (motor2_prev_A == HIGH and motor2_A == HIGH and motor2_prev_B == HIGH and motor2_B == LOW) or \
         (motor2_prev_A == HIGH and motor2_A == LOW and motor2_prev_B == LOW and motor2_B == LOW):
        with mutex_motor2_counts:
            motor2_counts -= 1 #anticlockwise
    

    motor2_prev_A = motor2_A
    motor2_prev_B = motor2_B 

    return None 




#------------ setup the robot and the controller --------
robot = DiffDriveRobot(inertia=5, dt=0.1, drag=1, wheel_radius=WHEEL_RAD, wheel_sep=WHEEL_SEP)
controller = RobotController(Kp=1,Ki=0.25,wheel_radius=WHEEL_RAD,wheel_sep=WHEEL_SEP)
planner = TentaclePlanner(dt=0.1,steps=5,alpha=1,beta=1e-5)

# ----------- setup pins ----------

# Set GPIO modes
GPIO.setmode(GPIO.BCM)

""" MOTOR 1 (left) """

GPIO.setup(PIN_MOTOR1_A_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_MOTOR1_B_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(PIN_MOTOR1_IN1, GPIO.OUT)
GPIO.setup(PIN_MOTOR1_IN2, GPIO.OUT)
GPIO.setup(PIN_MOTOR1_PWM_ENABLE, GPIO.OUT)

motor1_enable_pwm = GPIO.PWM(PIN_MOTOR1_PWM_ENABLE, 1000)


""" MOTOR 2 (right) """
GPIO.setup(PIN_MOTOR2_A_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_MOTOR2_B_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(PIN_MOTOR2_IN1, GPIO.OUT)
GPIO.setup(PIN_MOTOR2_IN2, GPIO.OUT)
GPIO.setup(PIN_MOTOR2_PWM_ENABLE, GPIO.OUT)

motor2_enable_pwm = GPIO.PWM(PIN_MOTOR2_PWM_ENABLE, 1000)


poses = []
velocities = []
duty_cycle_commands = []


time_interval = 0.5

while True:
    try:
        # calculate motor speed:
        # revs of the motor shaft:
        with mutex_motor1_counts:
            motor1_revs = motor1_counts/ (CPR * GEAR_RATIO)
            

            # convert revolutions to rpm:
            motor1_rpms = (motor1_revs / time_interval) * 60

            print(f"RPM = {motor1_rpms:.2f} RPM, count = {motor1_counts: .2f}")

            # reset the encoder count to zero (so the past encoder counts dont affect the calculation of the next rpm)
            motor1_counts = 0

        
        with mutex_motor2_counts:
            motor2_revs = motor2_counts/ (CPR * GEAR_RATIO)
            

            # convert revolutions to rpm:
            motor2_rpms = (motor2_revs / time_interval) * 60

            print(f"RPM = {motor2_rpms:.2f} RPM, count = {motor2_counts: .2f}")

            # reset the encoder count to zero (so the past encoder counts dont affect the calculation of the next rpm)
            motor1_counts = 0


        robot.wl = motor1_rpms*2*math.pi/60

        # update the actual robot.wr 
        robot.wr = motor2_rpms*2*math.pi/60

        
        v,w = planner.plan(goal_x,goal_y,goal_th,robot.x,robot.y,robot.th)
        
        duty_cycle_l,duty_cycle_r = controller.drive(v,w,robot.wl,robot.wr)
        
        # Simulate robot motion - send duty cycle command to robot
        x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r)
        
        # Log data
        poses.append([x,y,th])
        duty_cycle_commands.append([duty_cycle_l,duty_cycle_r])
        velocities.append([robot.wl,robot.wr])

        time.sleep(0.1)
            

    except KeyboardInterrupt:

        GPIO.cleanup()
        print("Done")
        break

