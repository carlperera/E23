import sys
sys.path.append("..")

from Robot import Robot
from Vision import Vision

from State import State, StartPosition

START_POINT = 0
END_POINT = 360
INCREMENTS = 30
ANTICLOCKWISE_CALIBRATION_INPUTS = list(range(START_POINT, END_POINT + 1, INCREMENTS))

if __name__ == "__main__":
    vision = None
    start_pos = StartPosition.RIGHT.value
    robot = Robot(vision=vision, start_pos=start_pos)
    
    for angle_test in ANTICLOCKWISE_CALIBRATION_INPUTS:
        input(f"Current value: {angle_test}. Press Enter to continue...")

        # robot.move_test(distance=0.1, speed=1.0)
        robot.rotate_anticlockwise_calibrate(angle_test, speed = 1.0)
    
        print(f"(x,y) = ({round(robot.x,2), round(robot.y,2)})  --- angle = {round(robot.th,4)}")
    

   
