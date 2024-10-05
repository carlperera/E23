from enum import Enum, auto



class State(Enum):
    # Start States
    START_LEFT = auto()  # start on the left (facing 3 o'clock)
    START_RIGHT = auto()  # start on the right (facing 9 o'clock)

    # rotate at the start position - to find a ball if exists
    ROTATING_START_LEFT_EXPLORE = auto()  # rotating at start to find a ball (rotating clockwise)
    ROTATING_START_RIGHT_EXPLORE = auto()  # rotating at start to find a ball (rotating anticlockwise)

    # if no ball found whilst rotating at start -> rotate to face centre -> go to centre 
    ROTATING_FACE_CENTRE_START_LEFT = auto()
    ROTATING_FACE_CENTRE_START_RIGHT = auto() 
    
    # if no ball/box found whilst rotating at start -> facing start now -> move to centre
    MOVE_TO_CENTRE_BALL = auto() 
    MOVE_TO_CENTRE_BOX = auto()

    # Rotating to face centre
    ROTATE_FACE_CENTRE_BALL = auto()

    # Rotating in place to "explore" (go full 360 by doing 180 twice)
    ROTATE_EXPLORE_FULL_PRIMARY_PART_1 = auto()
    ROTATE_EXPLORE_FULL_PRIMARY_PART_2 = auto()

    # Locked onto target 
    ROTATE_LEFT_TARGET = auto()  # locked onto target, just rotating to centre it in frame 
    ROTATE_RIGHT_TARGET = auto()  # locked onto target just rotating clockwise 
    MOVE_TO_TARGET = auto()  # moving to target but not close enough
    CLOSE_TO_TARGET = auto()  # close to target, switch to secondary camera? (<30% of top frame)
 
    # Secondary camera
    ROTATE_LEFT_TARGET_SECONDARY = auto()  # secondary - ball is left
    ROTATE_RIGHT_TARGET_SECONDARY = auto()  # secondary - ball is right
    MOVE_TO_TARGET_SECONDARY = auto()
    
    # Pickup Sequence
    PICKUP_BALL = auto()  # actuate the servo, stops the state machine to actuate claw, then lower it down

    # Rotating in secondary camera
    # ROTATE_EXPLORE_FULL_SECONDARY = auto()  # rotate 360 in secondary camera, if the ball has disappeared from view
    
    ROTATE_EXPLORE_FULL_SECONDARY_PART_1 = auto()
    ROTATE_EXPLORE_FULL_SECONDARY_PART_2 = auto()

    ROTATE_EXPLORE = auto()  # rotate in place 360 to find a ball if exists 



   

    # Delivery
    BOX_DETECT_START = auto()  # rotate in place 360 to see the box (at any (x,y) pos)
    ROTATE_FACE_CENTRE_BOX = auto()

    ROTATE_LEFT_BOX = auto()
    ROTATE_RIGHT_BOX = auto()
    MOVE_TO_BOX = auto()

    CLOSE_TO_BOX = auto()
    TURN_BACK_TO_BOX = auto()
    REVERSE_TO_BOX = auto()

    # if at centre 
    ROTATE_EXPLORE_BOX_PART_1 = auto()
    ROTATE_EXPLORE_BOX_PART_2 = auto()

    FLAP_SEQUENCE = auto()  # open the flap to let balls out into the collection box, then close the flap 
   

    @property
    def is_ball_state(self):
       return not self.is_box_state
    
    @property
    def is_box_state(self):
        return self in frozenset((
            State.BOX_DETECT_START, 
            State.ROTATE_FACE_CENTRE_BOX,
            State.ROTATE_LEFT_BOX,
            State.ROTATE_RIGHT_BOX,
            State.MOVE_TO_BOX, 
            State.CLOSE_TO_BOX,
            State.TURN_BACK_TO_BOX,
            State.REVERSE_TO_BOX,
            State.ROTATE_EXPLORE_BOX_PART_1,
            State.ROTATE_EXPLORE_BOX_PART_2,
            State.FLAP_SEQUENCE,
    ))

    @property
    def is_primary_state(self):
        return not self.is_secondary_state
    
    @property
    def is_secondary_state(self):
        return self in frozenset((
            State.ROTATE_LEFT_TARGET_SECONDARY,
            State.ROTATE_RIGHT_TARGET_SECONDARY,
            State.MOVE_TO_TARGET_SECONDARY,
        ))







class StartPosition(Enum):
    """
    Starting on LEFT
    Starting on RIGHT
    """
    LEFT = auto()  
    RIGHT = auto()
