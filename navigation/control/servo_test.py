# from gpiozero import AngularServo
# from time import sleep

# servo = AngularServo(10, min_pulse_width=0.0006, max_pulse_width=0.0023)

# while (True):
#     servo.angle = 90
#     sleep(2)
#     servo.angle = 0
#     sleep(2)
#     servo.angle = -90
#     sleep(2)

import gpiozero
from gpiozero import Motor
import time
import RPi.GPIO as GPIO

# Set GPIO modes
GPIO.setmode(GPIO.BCM)
motor_A_in1 = 2 #non pwm
motor_A_in2 = 3 # non pwm

motor_B_in1 = 17 # non pwm
motor_B_in2 = 27 # non pwm

# motor_B_in2 = 24
# motor_B_in1 = 25
# motor_B_en = 19

motor_enable = 18
motor_enable2 = 19

GPIO.setup(motor_A_in1, GPIO.OUT)
GPIO.setup(motor_A_in2, GPIO.OUT)

GPIO.setup(motor_B_in1, GPIO.OUT)
GPIO.setup(motor_B_in2, GPIO.OUT)
GPIO.setup(motor_enable, GPIO.OUT)
GPIO.setup(motor_enable2, GPIO.OUT)



enable_pwm = GPIO.PWM(motor_enable, 1000)
enable_pwm2 = GPIO.PWM(motor_enable2, 1000)

# enable_pwm.start(100) # Turning on both enables
# enable_pwm2.start(100)

#input 1 and input 2 pins used for motor A
#input 1 LOW and input 2 HIGH = forward
#input 1 HIGH and input 2 LOW = backwards
#input 3 and input 4 pins used for motor B
#input 3 LOW and input 4 HIGH = forward
#input 3 HIGH and input 4 LOW = backwards

def forwards(speed):
    GPIO.output(motor_A_in1, GPIO.HIGH)
    GPIO.output(motor_B_in1, GPIO.HIGH)
    GPIO.output(motor_A_in2, GPIO.LOW)
    GPIO.output(motor_B_in2, GPIO.LOW)
    enable_pwm.start(speed)
    enable_pwm2.start(speed)    

def backwards(speed):
    GPIO.output(motor_A_in2, GPIO.HIGH)
    GPIO.output(motor_B_in2, GPIO.HIGH)
    GPIO.output(motor_A_in1, GPIO.LOW)
    GPIO.output(motor_B_in1, GPIO.LOW)
    enable_pwm.start(speed)
    enable_pwm2.start(speed) 

def right(direction, speed):
    if direction == "f":
        GPIO.output(motor_A_in1, GPIO.HIGH)
        GPIO.output(motor_B_in1, GPIO.HIGH)
        GPIO.output(motor_A_in2, GPIO.LOW)
        GPIO.output(motor_B_in2, GPIO.LOW)
        enable_pwm.start(speed)
        enable_pwm2.start(speed/4) 

    else:
        GPIO.output(motor_A_in2, GPIO.HIGH)
        GPIO.output(motor_B_in2, GPIO.HIGH)
        GPIO.output(motor_A_in1, GPIO.LOW)
        GPIO.output(motor_B_in1, GPIO.LOW)
        enable_pwm.start(speed)
        enable_pwm2.start(speed/4) 

def left(direction, speed):
    if direction == "f":
        GPIO.output(motor_A_in1, GPIO.HIGH)
        GPIO.output(motor_B_in1, GPIO.HIGH)
        GPIO.output(motor_A_in2, GPIO.LOW)
        GPIO.output(motor_B_in2, GPIO.LOW)
        enable_pwm.start(speed/4)
        enable_pwm2.start(speed) 

    else:
        GPIO.output(motor_A_in2, GPIO.HIGH)
        GPIO.output(motor_B_in2, GPIO.HIGH)
        GPIO.output(motor_A_in1, GPIO.LOW)
        GPIO.output(motor_B_in1, GPIO.LOW)
        enable_pwm.start(speed/4)
        enable_pwm2.start(speed) 


while True:
    try:
        directionFlag = input("Please state first letter of direction and speed: ")
        direction = directionFlag.split(" ")[0]
        speed =  int(directionFlag.split(" ")[1])
        print(direction)

        if direction == "b": # if user types "back" change direction of motor
            backwards(speed)
                
        if direction == "f":
            forwards(speed)

        if direction == "bl":
            left("b", speed)
        if direction == "br":
            right("b", speed)
        if direction == "fl": # front left
            left("f", speed)
        if direction == "fr": # Front right
            right("f", speed)
        if direction == "bl":
            left("b", speed)
        if direction == "br":
            right("b", speed)

        if direction == "s":
            enable_pwm.stop()
            enable_pwm2.stop()

    except KeyboardInterrupt:
        GPIO.output(motor_A_in2, GPIO.LOW)
        GPIO.output(motor_B_in2, GPIO.LOW)
        GPIO.output(motor_A_in1, GPIO.LOW)
        GPIO.output(motor_B_in1, GPIO.LOW)
        enable_pwm.stop()
        enable_pwm2.stop()

        GPIO.cleanup()
        print("Done")
        break

