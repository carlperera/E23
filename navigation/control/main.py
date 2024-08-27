from Robot import Robot
from Vision import Vision
from green_ball_tracker import General_control

import numpy as np
import time
import gpiozero
import math
from threading import Thread, Lock
from functools import partial

from enum import Enum


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

WHEEL_SEP = (147 + 64)/1000
WHEEL_RAD = (53/2)/1000

CAMERA_FPS = 30
FRAME_SKIP = 1

# class State(Enum):
#     EXPLORE
#     ON_TARGET
#     RETURN

if __name__ == "__main__":

    robot = Robot()

    vision = Vision()

    while True: # for each frame from camera 

        # get latest frame
        ret, frame = vision.camera.read()

        if not ret:
            print("error")
            break

        # check for line detected - display on imshow


        # check for ball detected 
    
        


        # wait for the next frame - 30 FPS 
        time.sleep(1/CAMERA_FPS)



