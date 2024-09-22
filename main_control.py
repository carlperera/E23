from Robot import Robot
from Vision import Vision

from State import State, StartPosition


if __name__ == "__main__":
    # vision = Vision()
    vision = None
    start_pos = StartPosition.RIGHT.value
    robot = Robot(vision=vision, start_pos=start_pos)
   

    # robot.move_test(distance=0.1, speed=1.0)
    robot.rotate(360, speed = 0.5)
    
    # robot.move(distance=0.3, speed=1.0)

    print(f"(x,y) = ({round(robot.x,2), round(robot.y,2)})  --- angle = {robot.th}")
    # robot.move(distance=0.3)

   
