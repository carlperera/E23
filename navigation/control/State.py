from enum import Enum

class State(Enum):
    EXPLORE_START = 0 
    EXPLORE_ROTATING = 1 
    EXPLORE_CENTRE = 2
    START_ROTATE_TO_ = 3

    ON_TARGET = 4
    CLOSE_TO_TARGET = 5
    START_RETURN = 6
    RETURNING = 7
