from enum import Enum

class State(Enum):
    START = 0  # check if ball in frame, if so lock onto it, and rotate to centre it in frame 

    ROTATE_EXPLORE = 1  # rotating at start to find a ball if exists  (rotating anticlockwise)
    
    ROTATE_LEFT_TARGET = 2 # locked onto target, just rotating to centire it in frame 
    ROTATE_RIGHT_TARGET = 3  # locked onto target just rotating clockwise 

    ROTATE_TO_CENTRE = 4 
    MOVE_TO_CENTRE = 5

    MOVE_TO_TARGET = 7 # moving to target but not close enough
    CLOSE_TO_TARGET = 8  # close to target 
    START_RETURN = 9 # move back slightly
    ROTATE_TO_FACE_START = 10 # move to face the start point 
    MOVE_TO_START = 11 # move to the start, once arrvied at start, swtich back to start state 
    ROTATE_EXPLORE2 = 12
    ROTATE_FACE_CENTRE = 13
    ROTATE_EXPLORE_FULL = 14
    ROTATING_TO_FACE_START = 15

class StartPosition(Enum):
    """
    Starting on LEFT
        
    Starting on RIGHT

    """
    LEFT = 0  
    RIGHT = 1

"""
1) robot starts out either facing 
    - hidden parameter for now 
2) rotate 90 to get the full field of view 
    2.1) if spotted a ball, go to it
    2.2) else after full 90, rotate to 45 and go to centre of court 
3) 

4) 


"""
    