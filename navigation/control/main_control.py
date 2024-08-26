from  DiffDriveRobot import DiffDriveRobot
from RobotController import RobotController
from TentaclePlanner import TentaclePlanner

from Robot import Robot

import numpy as np
import time
import gpiozero
import math
from threading import Thread, Lock
from functools import partial

"""
whenever you stop the robot, you want to set steps for both motors to 0
  
to rotate clockwise: 
-> motor left driven forward
-> motor right driven backward 

to rotate anti-clockwise: 
-> motor left driven backward
-> motor right driven forward

to go straight:
-> motor left driven forward by count1 ratio 
-> motor right driven forward by count2 ratio

distance_travelled_forward = motor1_scaling*count + motor2_scaling*count
"""

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

"""
states = 
1.looking for ball to lock onto
2.locked onto ball, 
"""

if __name__ == "__main__":

    robot = Robot()
   
    robot.move_forward(distance=0.1, speed = 1.0)
    # robot.rotate(180, 0.5)


    # time_interval = (1/CAMERA_FPS)*FRAME_SKIP

    # while True:

    #     try:
    #         # get frame from camera 
    #         # check frame for tennis balls 
    #         # check frame for lines

    #         # if already locked onto ball 
    #         frame = 


    #         time.sleep(time_interval)
                

    #     except KeyboardInterrupt:

            
    #         print("Done")
    #         break