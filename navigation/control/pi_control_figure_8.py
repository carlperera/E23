from  DiffDriveRobot import DiffDriveRobot
from RobotController import RobotController
import numpy as np
# import matplotlib.pyplot as plt
# from IPython import display
import gpiozero
from gpiozero import Motor
import time
import RPi.GPIO as GPIO

"""
MOTOR 1 = LEFT 
MOTOR 2 = RIGHT
"""

#------------ setup the robot and the controller --------
wheel_sep = (147 + 64)/1000
wheel_rad = (53/2)/1000

robot = DiffDriveRobot(inertia=5, dt=0.1, drag=1, wheel_radius=wheel_rad, wheel_sep=wheel_sep)
controller = RobotController(Kp=1,Ki=0.25,wheel_radius=0.05,wheel_sep=0.15)

# ----------- setup pins ----------
# Set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
PIN_MOTOR_A_IN1 = 17
PIN_MOTOR_A_IN2 = 27

PIN_MOTOR_B_IN1 = 23
PIN_MOTOR_B_IN2 = 24    

PIN_MOTOR1_ENABLE = 18
PIN_MOTOR2_ENABLE = 10

# motor 1 outputs for feedback (yellow and white) - signal outputted by encoder into the pi pwm 
PIN_MOTOR1_OUT_A = 3
PIN_MOTOR1_OUT_B = 2 

PIN_MOTOR2_OUT_A = 5
PIN_MOTOR2_OUT_B = 6

# setup the motor 1 output pins
GPIO.setup(PIN_MOTOR1_OUT_A, GPIO.IN)
GPIO.setup(PIN_MOTOR1_OUT_B, GPIO.IN)

# setup the motor 2 output pins
GPIO.setup(PIN_MOTOR2_OUT_A, GPIO.IN)
GPIO.setup(PIN_MOTOR2_OUT_B, GPIO.IN)

# motor A in 
GPIO.setup(PIN_MOTOR_A_IN1, GPIO.OUT)
GPIO.setup(PIN_MOTOR_A_IN2, GPIO.OUT)

# motor B in 
GPIO.setup(PIN_MOTOR_B_IN1, GPIO.OUT)
GPIO.setup(PIN_MOTOR_B_IN2, GPIO.OUT)

# set pin modes
GPIO.setup(PIN_MOTOR1_ENABLE, GPIO.OUT)
GPIO.setup(PIN_MOTOR2_ENABLE, GPIO.OUT)

# motor1 pwms
motor1_pwm1 = GPIO.PWM(PIN_MOTOR_A_IN1, 1000)
motor1_pwm2 = GPIO.PWM(PIN_MOTOR_A_IN2, 1000)

# motor 2 pwms
motor2_pwm1 = GPIO.PWM(PIN_MOTOR_B_IN1, 1000)
motor2_pwm2 = GPIO.PWM(PIN_MOTOR_B_IN2, 1000)

# enable pins for both motors
motor1_enable_pwm = GPIO.PWM(PIN_MOTOR1_ENABLE, 1000)
motor2_enable_pwm = GPIO.PWM(PIN_MOTOR2_ENABLE, 1000)

# enable both motors
motor1_enable_pwm.start(100)  # enable pin on motor1 to high 
motor2_enable_pwm.start(100)  # enable pin on motor2 to high 

def get_duty_cycle(pin, timeout=1.0):
    # Variables to keep track of timing
    high_time = 0
    low_time = 0

    # Wait for the pin to go high
    start_time = time.time()
    while GPIO.input(pin) == GPIO.LOW:
        if time.time() - start_time >= timeout:
            return 0.0  # If the pin stays low for too long, return 0% duty cycle
        pass  # Wait for the rising edge
    high_start_time = time.time()  # Record the start time for high state

    # Measure high time
    while GPIO.input(pin) == GPIO.HIGH:
        if time.time() - high_start_time >= timeout:
            return 100.0  # If the pin stays high for too long, return 100% duty cycle
        pass  # Wait for the pin to go low
    high_time = time.time() - high_start_time  # Record the duration of high state

    # Measure low time
    low_start_time = time.time()  # Reset the start time for low state
    while GPIO.input(pin) == GPIO.LOW:
        if time.time() - low_start_time >= timeout:
            return (high_time / (high_time + low_time)) * 100  # If the pin stays low for too long, return the current duty cycle
        pass  # Wait for the pin to go high
    low_time = time.time() - low_start_time  # Record the duration of low state

    # Calculate the duty cycle
    total_time = high_time + low_time
    duty_cycle = (high_time / total_time) * 100  # Convert to percentage

    return duty_cycle


while True:
    try:
        poses = []
        velocities = []
        duty_cycle_commands = []

        for i in range(300):
    
            # update the actual robot.wl
            motor1_out_a_temp = get_duty_cycle(PIN_MOTOR1_OUT_A) 
            motor1_out_b_temp = get_duty_cycle(PIN_MOTOR1_OUT_B)

            motor2_out_a_temp = get_duty_cycle(PIN_MOTOR2_OUT_A)
            motor2_out_b_temp = get_duty_cycle(PIN_MOTOR2_OUT_B)
    
            robot.wr = (motor1_out_a_temp + motor1_out_b_temp)/4

            # update the actual robot.wr 
            robot.wl = (motor2_out_a_temp + motor2_out_b_temp)/4

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