from enum import Enum

class State(Enum):
    
    # Start States
    START_LEFT = 0  # start on the left (facing 3 o'clock)
    START_RIGHT = 1  # start on the right (facing 9 o'clock)

    # Start Rotatating
    ROTATING_START_LEFT_EXPLORE = 2  # rotating at start to find a ball if exists  (rotating clockwise)
    ROTATING_START_RIGHT_EXPLORE = 3 # rotating at start to find a ball if exists  (rotating anticlockwise)   

    # go to centre 
    ROTATING_FACE_CENTRE_START_LEFT = 4
    ROTATING_FACE_CENTRE_START_RIGHT = 5 
    MOVE_TO_CENTRE = 6 

    # rotating in place to "explore" (go full 360)
    ROTATE_EXPLORE_FULL_PRIMARY = 7

    # locked onto target 
    ROTATE_LEFT_TARGET = 8 # locked onto target, just rotating to centire it in frame 
    ROTATE_RIGHT_TARGET = 9  # locked onto target just rotating clockwise 
    MOVE_TO_TARGET = 10 # moving to target but not close enough
    CLOSE_TO_TARGET = 11  # close to target, switch to secondary camera? (<30% ??)

    # secondary camera
    ROTATE_LEFT_SECONDARY = 12
    ROTATE_RIGHT_SECONDARY = 13
    MOVE_TO_TARGET_SECONDARY = 14
    
    # Pickup Sequence
    PICKUP_BALL = 15   # actuate the servo, stops the state machine to actuate claw, then lower it down

    # PICKUP_IN_PROGRESS = 16  # Servo still lifting the ball up, start rotating to explore too at the same time (lock onto a different ball)
    # if the ball has disappaered from view, switch back to primary
    ROTATE_EXPLORE_FULL_SECONDARY = 17    # rotate 360 in secondary camera, if the ball has disappaered from view
  
    # Move to centre to rotate in place (explore)
    ROTATE_FACE_CENTRE_BALL = 18 # face the centre to move to it, if you want to spot balls or the collection box (a safe zone)
    ROTATE_EXPLORE = 19 # rotate in place 360 to find a ball if exists 

    # Delivery
    BOX_DETECT_START = 20 # rotate in place 360 to see the box (at any (x,y) pos)
        # otherwise if no box detected in current position, go to centre -> ROTATE_FACE_CENTRE
    ROTATE_FACE_CENTRE_BOX = 21

    MOVE_TO_BOX = 22
    CLOSE_TO_BOX  = 23
    TURN_BACK_TO_BOX = 24
    REVERSE_TO_BOX =  25

    FLAP_SEQUENCE = 26  # open the flap to let balls out into the collection box, then close the flap 





    







class StartPosition(Enum):
    """
    Starting on LEFT
    Starting on RIGHT
    """
    LEFT = 0  
    RIGHT = 1

"""
1) robot starts out either on left (facing 3 o'clock) or on right (facing 9 o'clock)

2) rotate full 90 to get the full field of view 
    2.1) if spotted a ball, go to it
    2.2) else after full 90, rotate to 45 and go to centre of court 
3) at centre of court, rotate a full 350
    3.1) if spotted a ball, go to it 
    3.2) if not spotted a ball after full 180, full somewhere and rotate again full 360

4) if locked onto ball
    4.1) rotate left 
    4.2) rotate right

5) once close to ball, switch to secondary camera
    5.1) adjust to left 
    5.2) go straight
    5.3) adjust to right 
    5.4) if ball disappears from view (gust of wind, etc), then rotate ful 360 using secondary camera to see if there is ball around 
    5.5) if ball not in view with secondary camera after full 360, then switch back to primary camera and go full 360 again 

6) once ball in claw (check using secondary primary)
    6.1) start servo
    6.2) once servo actuated to full extent (assume ball dropped into the tray) then start looking for the next ball while lowering the claw again

7) once ball picked up, go rotate a full 360 to spot another ball 

8) once number of balls in tray is >= DROPOFF_THRESHOLD



    


QUESTIONS:
    1) how to check if robot is within the bounds? 
        1.1) using odometry (check if current position is within)
            MIN_X < self.x < MAX_X and MIN_Y < self.y < MAX_Y
        1.2) using line detection? 
            -> more accurate but requires more CPU power 
    2) what if robot doesn't detect a ball when it's rotating in place at centre/any other place?
        2.1) if at centre and rotating in place, go to 
    

"""
    