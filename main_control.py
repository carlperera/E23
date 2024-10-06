from Robot import Robot
from Vision import Vision

from State import State, StartPosition


if __name__ == "__main__":
    # vision = Vision()
    vision = None
    start_pos = StartPosition.RIGHT.value
    robot = Robot(vision=vision, start_pos=start_pos)
   

    # robot.move_test(distance=2.0, speed=1.0)
    robot.rotate(180, speed = 0.5)

    # robot.forward_calibrate(distance=0.1, speed=1.0)
    # robot.move_forward(distance = 0.25, speed = 0.5)
    print(f"(x,y) = ({round(robot.x,2), round(robot.y,2)})  --- angle = {robot.th}")
    # robot.move(distance=0.3)

   
