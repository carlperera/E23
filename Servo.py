from enum import Enum
import RPi.GPIO as GPIO

from time import sleep
from enum import Enum

class SERVO_PINS(Enum):
    PIN_FLAP_PWM = 12
    PIN_CLAW_PWM = 18


class Servo:

    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50)
        self.pwm.start(0)
    
    def setAngle(self, angle):
        duty = angle / 18 + 3
        GPIO.output(self.pin, True)
        self.pwm.ChangeDutyCycle(duty)
        sleep(1)
        GPIO.output(self.pin, False)
        self.pwm.ChangeDutyCycle(duty)
        # self.pwm.ChangeDutyCycle(0)
    
    def stop(self):
        self.setAngle(0)
        self.pwm.stop()
        GPIO.cleanup()


class Flap(Servo):
    def __init__(self):
        super.__init__(SERVO_PINS.PIN_FLAP_PWM.value)

    def open(self):
        self.setAngle(20)
        sleep(1)
    
    def close(self):
        self.setAngle(135)
        sleep(1) 


    
class Claw(Servo):
    def __init__(self, pin):
        super.__init__(SERVO_PINS.PIN_CLAW_PWM.value)

    def open(self):
        self.setAngle(-15)
        sleep(1)
    
    def close(self):
        self.setAngle(155)
        sleep(1)


    
       
    
