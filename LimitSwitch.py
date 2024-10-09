""" USED as swap for the current ultrasonic sensor 

"""
import RPi.GPIO as GPIO
from enum import Enum, auto

# GPIO pin the limit switch is connected to 


class LIMIT_SWTICH_PINS(Enum):
    left_pin = 7
    right_pin = 8

class LimitSwitch:
    def __init__(self, pin_number):
        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
        GPIO.setwarnings(False)
        self.pin_number = pin_number
        # GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_number, GPIO.IN)
        
    def check_being_pressed(self):
        return GPIO.input(self.pin_number) == GPIO.HIGH
    





# import RPi.GPIO as GPIO
# import time

# # Set up the GPIO mode and pin
# GPIO.setmode(GPIO.BCM)  # Use physical pin numbers
# GPIO.setup(8, GPIO.IN)  # Use internal pull-down resistor
# GPIO.setup(7, GPIO.IN)

# try:
#     while True:
#         if GPIO.input(8) == GPIO.HIGH or GPIO.input(7) == GPIO.HIGH:
#             print("Switch Pressed (High)")
#         else:
#             print("Switch Released (Low)")
#         time.sleep(0.1)

# except KeyboardInterrupt:
#     print("Program stopped.")
#     GPIO.cleanup()  # Clean up GPIO settings when exiting