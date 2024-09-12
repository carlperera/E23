from gpiozero import AngularServo
from time import sleep
from enum import Enum
import RPi.GPIO as GPIO


class SERVO_PINS(Enum):
    PIN_SERVO1_PWM = 12

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PINS.PIN_SERVO1_PWM.value, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PINS.PIN_SERVO1_PWM.value, 50)
pwm.start(5)

def setAngle(angle):
    duty = angle / 18 + 3
    GPIO.output(SERVO_PINS.PIN_SERVO1_PWM.value, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(SERVO_PINS.PIN_SERVO1_PWM.value, False)
    pwm.ChangeDutyCycle(duty)

# try:
#     while True:
#         pwm.ChangeDutyCycle(5)
#         sleep(1) # sleep 1 second
#         pwm.ChangeDutyCycle(7.5) 
#         sleep(1) # sleep 1 second
#         pwm.ChangeDutyCycle(10) 
#         sleep(1) # sleep 1 second 
# except KeyboardInterrupt:
#     pwm.stop()
#     GPIO.cleanup()
def rotate_90():
    try:
        while True:
            setAngle(30)
            sleep(1) # sleep 1 second
            setAngle(135)
            sleep(1) # sleep 1 second
    except KeyboardInterrupt:
        pwm.stop()
        GPIO.cleanup()

def rotate_180():
    try:
        while True:
            setAngle(0)
            sleep(1)
            setAngle(180)
            sleep(1)
    except KeyboardInterrupt:
        pwm.stop()
        GPIO.cleanup()

# rotate_90() 
rotate_180()
# servo = AngularServo(SERVO_PINS.PIN_SERVO1_PWM.value, min_pulse_width=0.0006, max_pulse_width=0.0031)

# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(SERVO_PINS.PIN_SERVO1_PWM.value, GPIO.OUT)
# pwm = GPIO.PWM(SERVO_PINS.PIN_SERVO1_PWM.value, 50)
# pwm.start(0)

# # while (True):
# #     servo.angle = 0
# #     sleep(2)
# #     servo.angle = 90
# #     sleep(2)
# #     servo.angle = 180
# #     sleep(2)