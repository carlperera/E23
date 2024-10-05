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
    
    # for dist in FORWARD_CALIBRATION_INPUTS:
    #     input(f"Current value: {dist}. Press Enter to continue...")

    #     # robot.move_test(distance=0.1, speed=1.0)
    #     robot.forward_calibrate(distance=dist, speed=0.5)
    
    #     print(f"(x,y) = ({round(robot.x,2), round(robot.y,2)})  --- angle = {round(robot.th,4)}")

    
    last_dist_test = None
    i = 0
    
    while i < len(FORWARD_CALIBRATION_INPUTS):
        dist_test = FORWARD_CALIBRATION_INPUTS[i]
        
        if last_dist_test is not None:
            print(f"Last value: {last_angle_test}.")
        
        user_input = input(f"Current value: {dist_test}. Press Enter to continue or 'r' to redo the last command...")

        if user_input.lower() == 'r' and last_angle_test is not None:
            dist_test = last_angle_test
            print(f"Redoing the last command with value: {dist_test}")
        else:
            last_angle_test = dist_test
            i += 1

        # robot.move_test(distance=0.1, speed=1.0)
        robot.forward_calibrate(distance = dist_test, speed=0.5)
    
        print(f"(x,y) = ({round(robot.x,2)}, {round(robot.y,2)})  --- angle = {round(robot.th,4)}")