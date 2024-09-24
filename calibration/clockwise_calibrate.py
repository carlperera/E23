import sys
sys.path.append("..")

from Robot import Robot
from Vision import Vision

from State import State, StartPosition

START_POINT = 0
END_POINT = 360
INCREMENTS = 30
CLOCKWISE_CALIBRATION_INPUTS = list(range(START_POINT, END_POINT + 1, INCREMENTS))

if __name__ == "__main__":
    vision = None
    start_pos = StartPosition.RIGHT.value
    robot = Robot(vision=vision, start_pos=start_pos)
    
    last_angle_test = None
    i = 0
    
    while i < len(CLOCKWISE_CALIBRATION_INPUTS):
        angle_test = CLOCKWISE_CALIBRATION_INPUTS[i]
        
        if last_angle_test is not None:
            print(f"Last value: {last_angle_test}.")
        
        user_input = input(f"Current value: {angle_test}. Press Enter to continue or 'r' to redo the last command...")

        if user_input.lower() == 'r' and last_angle_test is not None:
            angle_test = last_angle_test
            print(f"Redoing the last command with value: {angle_test}")
        else:
            last_angle_test = angle_test
            i += 1

        # robot.move_test(distance=0.1, speed=1.0)
        robot.rotate_clockwise_calibrate(angle_test, speed=0.5)
    
        print(f"(x,y) = ({round(robot.x,2)}, {round(robot.y,2)})  --- angle = {round(robot.th,4)}")