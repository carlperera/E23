from gpiozero import AngularServo
from time import sleep
from enum import Enum

class SERVO_PINS(Enum):
    PIN_SERVO1_PWM = 12
    # PIN_SERVO1_POS =  ,
    # PIN_SERVO1_GND = 

servo = AngularServo(SERVO_PINS.PIN_SERVO1_PWM.value, min_pulse_width=0.0006, max_pulse_width=0.0023)

while (True):
    servo.angle = 90
    sleep(1)
    # servo.angle = 80
    # sleep(1)
    # servo.angle = 70
    # sleep(1)
    # servo.angle = 60
    # sleep(1)
    servo.angle = -90
    sleep(1)
    # sleep(1)
    # servo.angle = -60
    # sleep(1)
    # servo.angle = -70
    # sleep(1)
    # servo.angle = -80
    # sleep(1)
    # servo.angle = -90
    # sleep(1)




    # servo.angle = 90
    # sleep(0.02)
    # servo.angle = 80
    # sleep(0.02)
    # servo.angle = 70
    # sleep(0.02)
    # servo.angle = 60
    # sleep(0.02)
    # servo.angle = 0
    # sleep(0.02)
    # servo.angle = -60
    # sleep(0.02)
    # servo.angle = -70
    # sleep(0.02)
    # servo.angle = -80
    # sleep(0.02)
    # servo.angle = -90
    # sleep(0.02)