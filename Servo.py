from enum import Enum
import RPi.GPIO as GPIO

from time import sleep
from enum import Enum

class SERVO_PINS(Enum):
    PIN_FLAP_PWM = 12
    PIN_CLAW_PWM1 = 13
    PIN_CLAW_PWM2 = 19

class Servo:

    def __init__(self, pin):
        self.pin = pin
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
        super().__init__(SERVO_PINS.PIN_FLAP_PWM.value)

    def open(self):
        self.setAngle(20)
        sleep(1)
    
    def close(self):
        self.setAngle(135)
        sleep(1) 

# class Claw(Servo):
#     def __init__(self):
#         super().__init__(SERVO_PINS.PIN_CLAW_PWM.value)

#     def open(self):
#         self.setAngle(-15)
#         sleep(1)
    
#     def close(self):
#         self.setAngle(155)
#         sleep(1)

class Claw:

    def __init__(self):
        self.pin1 = SERVO_PINS.PIN_CLAW_PWM1.value
        self.pin2 = SERVO_PINS.PIN_CLAW_PWM2.value

        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)

        self.pwm1 = GPIO.PWM(self.pin1, 50)
        self.pwm1.start(0)

        self.pwm2 = GPIO.PWM(self.pin2, 50)
        self.pwm2.start(0)
    
    def setAngle(self, angle):
        duty = angle / 18 + 3
        GPIO.output(self.pin1, True)
        GPIO.output(self.pin2, True)
        self.pwm1.ChangeDutyCycle(duty)
        self.pwm2.ChangeDutyCycle(duty)
        sleep(0.005)
        GPIO.output(self.pin1, False)
        GPIO.output(self.pin2, False)
        self.pwm1.ChangeDutyCycle(duty)
        self.pwm2.ChangeDutyCycle(duty)
        sleep(0.005)
      
    
    def stop(self):
        self.setAngle(0)
        self.pwm1.stop()
        self.pwm2.stop()
        GPIO.cleanup()

    def open(self):
        self.setAngle(-15)
        sleep(1)
    
    def close(self):
        self.setAngle(155)
        sleep(0)