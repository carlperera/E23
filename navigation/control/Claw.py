from enum import Enum

class SERVO_PINS(Enum):
    PIN_SERVO_PWM = 13

class Claw:    
    def __init__(self, pin = SERVO_PINS.PIN_SERVO_PWM.value):
        self.pin = pin 
        return None
    
    def start_pickup(self):
        return None

    def stop_pickup(self):
        return None

    def start_release(self):
        return None

    def stop_release(self):
        return None


