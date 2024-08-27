from enum import Enum

class State(Enum):
    START = 0  # check if ball in frame, if so lock onto it, and rotate to centre it in frame 
    START_ROTATING_TO_TARGET = 1  # locked onto ball in stationary, need to start rotating
    ROTATING_TO_TARGET = 2 # rotating to locked ball  
    START_FORWARD = 4 # locked onto ball, move forward, if locked ball falls outside bounds, stop moving, and go to ROTATING_TO_TARGET
    KEEP_FORWARD = 5  
    
    ROTATE_START = 5 # start rotating at start position to find a tennis ball if visible
    ROTATING = 6 # rotating to find a target (if exists), check if full 360 performed, if so STOP and go to middle of court 


    
    CLOSING_ONTO_TARGET = 8

    EXPLORE_CENTRE = 2
    START_ROTATE_TO_ = 3

    ON_TARGET = 4
    CLOSE_TO_TARGET = 5
    START_RETURN = 6
    RETURNING = 7
