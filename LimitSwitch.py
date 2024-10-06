""" USED as swap for the current ultrasonic sensor 

"""
import RPi.GPIO as GPIO

# GPIO pin the limit switch is connected to 
LIMIT_SWITCH_PIN = 17  

class LimitSwitch:
    def __init__(self):
        self.pin = LIMIT_SWITCH_PIN
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    def check_being_pressed(self):
        return GPIO.input(LIMIT_SWITCH_PIN) == GPIO.LOW
    



