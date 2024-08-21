


# while True:
#     command = input("Enter your command: ")


import cv2
import numpy as np

from green_ball_tracker import  General_control,Region_number

detector = General_control()
detector.track_balls()  

# while True:
#     if detector.max_ball.x is not None:
#         print(detector.max_ball.x)


    