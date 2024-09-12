from enum import Enum
import RPi.GPIO as GPIO

class FLAP_PINS(Enum):
    PIN_FLAP_PWM = 12

class Flap:    
    def __init__(self, pin = FLAP_PINS.PIN_FLAP_PWM):
        self.pin = pin 

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PINS.PIN_SERVO1_PWM.value, GPIO.OUT)
 
        pwm = GPIO.PWM(SERVO_PINS.PIN_SERVO1_PWM.value, 50)
      
        


    def start_open(self):

    def stop_open(self):


    def start_close(self):

    def stop_close(self):




from gpiozero import AngularServo
from time import sleep
from enum import Enum
import RPi.GPIO as GPIO

class SERVO_PINS(Enum):
    PIN_SERVO1_PWM = 12
    PIN_SERVO2_PWM = 13



def setAngle(pwm,angle):
    duty = angle / 18 + 3
    GPIO.output(SERVO_PINS.PIN_SERVO1_PWM.value, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(SERVO_PINS.PIN_SERVO1_PWM.value, False)
    pwm.ChangeDutyCycle(duty)


def rotate_90():
    try:
        while True:
            setAngle(pwm1, 30)
            sleep(1) # sleep 1 second
            setAngle(pwm1, 135)
            sleep(1) # sleep 1 second
    except KeyboardInterrupt:
        pwm1.stop()
        GPIO.cleanup()

def rotate_180():
    try:
        while True:
            setAngle(pwm2,0)
            sleep(1)
            setAngle(pwm2,180)
            sleep(1)
    except KeyboardInterrupt:
        pwm1.stop()
        GPIO.cleanup()






