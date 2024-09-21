from Robot import Robot
from Vision import Vision
from green_ball_tracker import General_control

import numpy as np
import time
import gpiozero
import math
from threading import Thread, Lock
from functools import partial
from State import State, StartPosition
import cv2

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
1.) exploring
    1.1) rotate around 360 until you find a ball in the frame, if ball found at any point, 
        1.1.1) lock onto it 
        1.1.2) go to 1.2
    1.2) go to centre of quadrant 
        1.1.1) do 1.1

2) locked on 
    2.1) start moving to target 
    2.2) keep moving to target
    2.3) close to target (top of tennis ball moves below horizontal threshold in camera), switch to slow mode 
    2.4) use ultrasonics to touch the ball and register a print out 

3) return to start 
    3.1) start moving to start by moving away from robot and rotating to face the the start location 
    3.2) keep moving to start 
    3.3) finish moving to start by slowing down to get more accuracy 
    3.4) finalise anything necessary 
"""

if __name__ == "__main__":
    count_frames = 0

    vision = Vision()
   
    time.sleep(2) # give it some time to setup the primary and secondary cameras 

    start_pos = StartPosition.LEFT
    robot = Robot(vision=vision, start_pos = start_pos)

    simulation_time_s = 10*60
    time_start = time.time()
    while True: # for each frame from camera 
        try:
            # get latest frame
            
            if robot.state == State.CLOSE_TO_TARGET:
                ret, frame = vision.camera_secondary.read()
            else:
                ret, frame = vision.camera_primary.read()
                
            if not ret:
                print("error")
                break
            # cv2.imshow("Webcam", frame)
            # ---------------------------- END SIMULATION ----------------------------
            if time.time() - time_start > simulation_time_s: 
                print("SIMULATION TIME REACHED: stopped the robot)")
                robot.shutdown()
                break
    
            robot.handle_state(frame)
            # cv2.imshow('Live feed',frame)
            
        
            # wait for the next frame - 30 FPS 
            time.sleep(1/CAMERA_FPS)
            # if cv2.waitKey(1) & 0xFF is ord('q'):
            #     break
        except KeyboardInterrupt:
            print("keyword interrupt")
            break

            



