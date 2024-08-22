from  DiffDriveRobot import DiffDriveRobot
from RobotController import RobotController
import numpy as np
import time
import RPi.GPIO as GPIO
import math

CPR = 48
GEAR_RATIO = 74.83

# ----------- CONSTANTS -------------

# MOTOR 1 (left)
PIN_MOTOR1_IN1 = 17 # LOW
PIN_MOTOR1_IN2 = 27 # LOW 
PIN_MOTOR1_PWM_ENABLE = 18 # LOW
PIN_MOTOR1_A_OUT = 21# LOW 
PIN_MOTOR1_B_OUT = 20 # LOW 

# MOTOR 2 (right)
PIN_MOTOR2_IN1 = 23 # LOW - good
PIN_MOTOR2_IN2 = 24 # LOW -good 
PIN_MOTOR2_PWM_ENABLE = 9 # LOW - good 
PIN_MOTOR2_A_OUT = 14# LOW - good 
PIN_MOTOR2_B_OUT = 15 # LOW - good

# GPIO - LOW AND HIGH
LOW = GPIO.LOW
HIGH = GPIO.HIGH

WHEEL_SEP = (147 + 64)/1000
WHEEL_RAD = (53/2)/1000


#------------ setup the robot and the controller --------
robot = DiffDriveRobot(inertia=5, dt=0.1, drag=1, wheel_radius=WHEEL_RAD, wheel_sep=WHEEL_SEP)
controller = RobotController(Kp=1,Ki=0.25,wheel_radius=WHEEL_RAD,wheel_sep=WHEEL_SEP)

# ----------- setup pins ----------

# Set GPIO modes
GPIO.setmode(GPIO.BCM)

""" MOTOR 1 (left) """

GPIO.setup(PIN_MOTOR1_A_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_MOTOR1_B_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(PIN_MOTOR1_IN1, GPIO.OUT)
GPIO.setup(PIN_MOTOR1_IN2, GPIO.OUT)
GPIO.setup(PIN_MOTOR1_PWM_ENABLE, GPIO.OUT)

motor1_enable_pwm = GPIO.PWM(PIN_MOTOR1_PWM_ENABLE, 1000)
motor1_enable_pwm.start(100) # initial speed

GPIO.output(PIN_MOTOR1_IN1, GPIO.HIGH)
GPIO.output(PIN_MOTOR1_IN2, GPIO.LOW)

""" MOTOR 2 (right) """
GPIO.setup(PIN_MOTOR2_A_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_MOTOR2_B_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(PIN_MOTOR2_IN1, GPIO.OUT)
GPIO.setup(PIN_MOTOR2_IN2, GPIO.OUT)
GPIO.setup(PIN_MOTOR2_PWM_ENABLE, GPIO.OUT)

motor2_enable_pwm = GPIO.PWM(PIN_MOTOR2_PWM_ENABLE, 1000)
motor2_enable_pwm.start(100) # initial speed

GPIO.output(PIN_MOTOR2_IN1, GPIO.HIGH)
GPIO.output(PIN_MOTOR2_IN2, GPIO.LOW)


while True:
    try:
        poses = []
        velocities = []
        duty_cycle_commands = []

        for i in range(300):
    
            motor1_rpms = 
            motor2_rpms = 
    
            robot.wl = motor1_rpms*2*math.pi/60



            # update the actual robot.wr 
            robot.wl = motor2_rpms*2*math.pi/60

            if i < 100: # drive in circular path (turn left) for 10 s
                duty_cycle_l,duty_cycle_r = controller.drive(0.1,1,robot.wl,robot.wr)
            elif i < 200: # drive in circular path (turn right) for 10 s
                duty_cycle_l,duty_cycle_r = controller.drive(0.1,-1,robot.wl,robot.wr)
            else: # stop
                duty_cycle_l,duty_cycle_r = (0,0)
            
            # Simulate robot motion - send duty cycle command to robot - simulated one 
            x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r)

            print(f"{x},{y},{th}")
            # actual drive - turn duty_cycle_l, duty_cycle_l into pwm signals and feed as inputs into the encoder inputs
            if duty_cycle_l > 0:
                motor1_pwm1.start(duty_cycle_l)
            else:
                motor1_pwm2.start(abs(duty_cycle_l))

            if duty_cycle_r > 0:
                motor2_pwm1.start(duty_cycle_r)
            else:
                motor2_pwm2.start(abs(duty_cycle_r))


            # Log data
            poses.append([x,y,th])
        
            duty_cycle_commands.append([duty_cycle_l,duty_cycle_r])
            velocities.append([robot.wl,robot.wr])

            time.sleep(0.1)
            

    except KeyboardInterrupt:

        motor1_pwm1.stop()
        motor1_pwm2.stop()
        motor2_pwm1.stop()
        motor2_pwm2.stop()

        GPIO.cleanup()
        print("Done")
        break