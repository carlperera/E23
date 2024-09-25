import sys
sys.path.append("..")

from Robot import Robot
from Vision import Vision

from State import State, StartPosition

# THESE ARE IN CM
START_POINT = 50
END_POINT = 645
INCREMENTS = 20

# CONVERT TO METRES HERE
FORWARD_CALIBRATION_INPUTS = [(i/100)/2 for i in list(range(START_POINT, END_POINT + 1, INCREMENTS*2))]

if __name__ == "__main__":
    vision = None
    start_pos = StartPosition.RIGHT.value
    robot = Robot(vision=vision, start_pos=start_pos)
    
    for dist in FORWARD_CALIBRATION_INPUTS:
        input(f"Current value: {dist}. Press Enter to continue...")

        # robot.move_test(distance=0.1, speed=1.0)
        robot.forward_calibrate(distance=dist, speed=1.0)
    
        print(f"(x,y) = ({round(robot.x,2), round(robot.y,2)})  --- angle = {round(robot.th,4)}")