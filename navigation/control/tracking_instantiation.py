


# while True:
#     command = input("Enter your command: ")


import cv2
import numpy as np

from green_ball_tracker import  General_control,Region_number

detector = General_control()
detector.track_balls()

# detector.track_balls(camNum=2)


# while True:
#     detector.track_balls()
#     print(detector.inCentre)





    