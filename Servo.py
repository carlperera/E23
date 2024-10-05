from enum import Enum
import RPi.GPIO as GPIO

from time import sleep
from enum import Enum

class CLAW_PINS(Enum):
    # PIN_FLAP_PWM = 12 # TODO: what's the pin for the flap? # TODO: does flap also need 2 pins or nah
    PIN_CLAW_PWM1 = 12
    PIN_CLAW_PWM2 = 13 # Originally 19, 12 for testing

class FLAP_PINS(Enum):
    PIN_FLAP_PWM1 = 19
    PIN_FLAP_PWM2 = 17

# ------------------------------ SERVO -----------------------------------
# class Servo:

#     def __init__(self, pin1, pin2, servo1_start_pos, servo2_start_pos):
#         self.pin1 = pin1
#         self.pin2 = pin2

#         GPIO.setup(self.pin1, GPIO.OUT)
#         GPIO.setup(self.pin2, GPIO.OUT)

#         self.pwm1 = GPIO.PWM(self.pin1, 50)
#         self.pwm1.start(0)

#         self.pwm2 = GPIO.PWM(self.pin2, 50)
#         self.pwm2.start(0)

#         self.setAngle(servo1_start_pos, self.pwm1)
#         self.setAngle(servo2_start_pos, self.pwm2)
#         # self.setAngle(170,self.pwm2) # Servo 1 needs to start in the fully turned direction

#     def setAngle(self, angle, servo):
#         duty = angle / 18 + 3
#         servo.start(duty)
    
#     def stop(self):
#         self.setAngle(0, self.pwm1)
#         self.setAngle(0, self.pwm2)
#         self.pwm1.stop()
#         self.pwm2.stop()
#         GPIO.cleanup()

#     def open(self):
#         self.setAngle(0, self.pwm2) # Make servo 1 go back to starting pos
#         self.setAngle(175, self.pwm1) # Make servo 2 go to 180 degrees
#         sleep(1)
    
#     def close(self):
#         angle1 = 0
#         angle2 = 175

#         while angle1 < 170 and angle2 > 0:
#             angle1+=1
#             angle2 -= 1
#             self.setAngle(angle1,self.pwm2) # Make servo 1 go to 180 degrees
#             self.setAngle(angle2, self.pwm1) # Make servo 2 go back to starting pos
#             sleep(0.0114)

#     def collect_ball(self):
#         self.open()
#         self.close()

# class Flap(Servo):
#     def __init__(self):
#         super().__init__(CLAW_PINS.PIN_FLAP_PWM.value)

#     def open(self):
#         self.setAngle(20)
#         sleep(1)
    
#     def close(self):
#         self.setAngle(135)
#         sleep(1) 

#     def deposit_balls(self):
#         self.open()
#         self.close()

# class Claw(Servo):
#     def __init__(self):
#         super().__init__(SERVO_PINS.PIN_CLAW_PWM.value)

#     def open(self):
#         self.setAngle(-15)
#         sleep(1)
    
#     def close(self):
#         self.setAngle(155)
#         sleep(1)


class Flap:

    def __init__(self):
        self.pin1 = FLAP_PINS.PIN_FLAP_PWM1.value
        # self.pin2 = FLAP_PINS.PIN_FLAP_PWM2.value

        GPIO.setup(self.pin1, GPIO.OUT)
        # GPIO.setup(self.pin2, GPIO.OUT)

        self.pwm1 = GPIO.PWM(self.pin1, 50)
        self.pwm1.start(0)

        # self.pwm2 = GPIO.PWM(self.pin2, 50)
        # self.pwm2.start(0)

        # self.setAngle(110,self.pwm1) # TODO: test -> fill this in: servo 1 needs to start in the fully turned direction

    def setAngle(self, angle, servo):
        duty = angle / 18 + 3 # TODO: change this for the flap
        servo.start(duty)

    def stop(self):
        self.setAngle(0, self.pwm1) 
        # self.setAngle(0, self.pwm2)
        self.pwm1.stop()
        # self.pwm2.stop()
        GPIO.cleanup()

    def open(self):
        # self.setAngle(0, self.pwm2) # TODO: Make servo 1 go back to starting pos
        self.setAngle(20, self.pwm1) # TODO: Make servo 2 go to 180 degrees

    
    def close(self):
        
        angle = 20  # TODO: change this 

        while angle < 110: # TODO: change this 
            
            angle += 1
            # self.setAngle(angle1,self.pwm2) # Make servo 1 go to 180 degrees
            self.setAngle(angle, self.pwm1) # Make servo 2 go back to starting pos
            sleep(0.0114) 

    def deposit_balls(self):
        # TODO: how to know if all the balls have been deposited 
        self.open()
        sleep(3)
        self.close()



class Claw:

    def __init__(self):
        self.pin1 = CLAW_PINS.PIN_CLAW_PWM1.value
        self.pin2 = CLAW_PINS.PIN_CLAW_PWM2.value
        # self.channel_list = (self.pin1, self.pin2)

        self.open_angle_1 = 160
        self.open_angle_2 = 10

        self.close_angle_1 = 0
        self.close_angle_2 = 170

        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)

        self.pwm1 = GPIO.PWM(self.pin1, 50)
        self.pwm1.start(0)

        self.pwm2 = GPIO.PWM(self.pin2, 50)
        self.pwm2.start(0)

        self.setAngle(170,self.pwm2) # Servo 1 needs to start in the fully turned direction

        # self.close_angle = 175

    def setAngle(self, angle, servo):
        duty = angle / 18 + 3
        servo.start(duty)
        # self.pwm2.start(duty)
        # sleep(1)
        # GPIO.output(self.channel_list, True)
        # # GPIO.output(self.pin2, True)
        # self.pwm1.ChangeDutyCycle(duty)
        # self.pwm2.ChangeDutyCycle(duty)
        # sleep(0.005)
        # GPIO.output(self.channel_list, False)
        # # GPIO.output(self.pin2, False)
        # self.pwm1.ChangeDutyCycle(duty)
        # self.pwm2.ChangeDutyCycle(duty)
        # sleep(0.005)
      
    
    def stop(self):
        self.setAngle(0, self.pwm1)
        self.setAngle(0, self.pwm2)
        self.pwm1.stop()
        self.pwm2.stop()
        GPIO.cleanup()

    def open(self):
        self.setAngle(self.open_angle_1, self.pwm1) # Make servo 1 go back to starting pos 
        self.setAngle(self.open_angle_2, self.pwm2) # Make servo 2 go to 180 degrees
        
        # sleep(1)
    
    def close(self):
        # pwm2 needs to return to 170
        # pwm1 needs to return to 0
        angle1 = self.open_angle_1 # the open angle for pwm1
        angle2 = self.open_angle_2 # the open angle for pwm2

        while angle1 > self.close_angle_1 and angle2 < self.close_angle_2: # pwm1 returns to 0 deg, pwm2 returns to 170 deg
            angle1 -=1 
            angle2 += 1
            self.setAngle(angle1,self.pwm1) # Make servo 1 go to 180 degrees
            self.setAngle(angle2, self.pwm2) # Make servo 2 go back to starting pos
            sleep(0.0114)
        sleep(1)

    def collect_ball(self):
        self.open()
        sleep(2)
        self.close()
        
        
