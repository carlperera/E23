


# while True:
#     command = input("Enter your command: ")


import cv2
import numpy as np
from test_double_cam import Vision
import time

from green_ball_tracker import  General_control,Region_number

# detector = General_control()
# detector.track_balls()



vision = Vision()
CAMERA_FPS = 30

while True: # for each frame from camera 
    try:
        # get latest frame
        curr_state = input("Please enter current state, 1 or 2:\n")
        
        if curr_state == "2":
            ret, frame = vision.camera_secondary.read()
        else:
            ret, frame = vision.camera_primary.read()
            
        if not ret:
            print("error")
            break
        cv2.imshow("Webcam", frame)
        # ---------------------------- END SIMULATION ----------------------------
        
        
    
        # wait for the next frame - 30 FPS 
        time.sleep(1/CAMERA_FPS)
        # if cv2.waitKey(1) & 0xFF is ord('q'):
        #     break
    except KeyboardInterrupt:
        print("keyword interrupt")
        break





    