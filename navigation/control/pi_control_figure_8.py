import DiffDriveRobot
import RobotController
import numpy as np
import matplotlib.pyplot as plt
from IPython import display
import gpiozero
from gpiozero import Motor
import time
import RPi.GPIO as GPIO
from duty_cycle import get_duty_cycle

"""
MOTOR 1 = LEFT 
MOTOR 2 = RIGHT
"""

type FILL_IN = any 

#------------ setup the robot and the controller --------
robot = DiffDriveRobot(inertia=5, dt=0.1, drag=1, wheel_radius=0.05, wheel_sep=0.15)
controller = RobotController(Kp=1,Ki=0.25,wheel_radius=0.05,wheel_sep=0.15)

# ----------- setup pins ----------
# Set GPIO modes
GPIO.setmode(GPIO.BCM)
PIN_MOTOR_A_IN1 = 17
PIN_MOTOR_A_IN2 = 27

PIN_MOTOR_B_IN1 = 23
PIN_MOTOR_B_IN2 = 24    

PIN_MOTOR1_ENABLE = 18
PIN_MOTOR2_ENABLE = 10

# motor 1 outputs for feedback (yellow and white) - signal outputted by encoder into the pi pwm 
PIN_MOTOR1_OUT_A = 10
PIN_MOTOR1_OUT_B = 12

PIN_MOTOR2_OUT_A = 2
PIN_MOTOR2_OUT_B = 3

# setup the motor 1 output pins
GPIO.setup(PIN_MOTOR1_OUT_A, GPIO.IN)
GPIO.setup(PIN_MOTOR1_OUT_B, GPIO.IN)

# setup the motor 2 output pins
GPIO.setup(PIN_MOTOR2_OUT_A, GPIO.IN)
GPIO.setup(PIN_MOTOR2_OUT_B, GPIO.IN)

motor1_out_A = GPIO.PWM(PIN_MOTOR1_OUT_A, 1000)
motor1_out_B = GPIO.PWM(PIN_MOTOR1_OUT_B, 1000)

motor2_out_A = GPIO.PWM(PIN_MOTOR2_OUT_A, 1000)
motor2_out_B = GPIO.PWM(PIN_MOTOR2_OUT_B, 1000)

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


# Test 1 - move the robot with figure 8s
plt.figure(figsize=(15,9))

poses = []
velocities = []
duty_cycle_commands = []
for i in range(300):

    # update the actual robot.wl
    robot.wr = (get_duty_cycle(motor1_out_A) + get_duty_cycle(motor1_out_B))/4

    # update the actual robot.wr 
    robot.wl = (get_duty_cycle(motor2_out_A) + get_duty_cycle(motor2_out_B))/4

    if i < 100: # drive in circular path (turn left) for 10 s
        duty_cycle_l,duty_cycle_r = controller.drive(0.1,1,robot.wl,robot.wr)
    elif i < 200: # drive in circular path (turn right) for 10 s
        duty_cycle_l,duty_cycle_r = controller.drive(0.1,-1,robot.wl,robot.wr)
    else: # stop
        duty_cycle_l,duty_cycle_r = (0,0)
    
    # Simulate robot motion - send duty cycle command to robot - simulated one 
    x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r)
    
    # actual drive - turn duty_cycle_l, duty_cycle_l into pwm signals and feed as inputs into the encoder inputs 
    motor1_pwm1.start(duty_cycle_r)

    motor2_pwm1.start(duty_cycle_l)

    

    # Log data
    poses.append([x,y,th])
    duty_cycle_commands.append([duty_cycle_l,duty_cycle_r])
    velocities.append([robot.wl,robot.wr])
    
    # Plot robot data
    plt.clf()
    plt.cla()
    plt.subplot(1,2,1)
    plt.plot(np.array(poses)[:,0],np.array(poses)[:,1])
    plt.plot(x,y,'k',marker='+')
    plt.quiver(x,y,0.1*np.cos(th),0.1*np.sin(th))
    plt.xlim(-1,1)
    plt.ylim(-1,1)
    plt.xlabel('x-position (m)')
    plt.ylabel('y-position (m)')
    plt.grid()
    
    plt.subplot(2,2,2)
    plt.plot(np.arange(i+1)*robot.dt,np.array(duty_cycle_commands))
    plt.xlabel('Time (s)')
    plt.ylabel('Duty cycle')
    plt.grid()
    
    plt.subplot(2,2,4)
    plt.plot(np.arange(i+1)*robot.dt,np.array(velocities))
    plt.xlabel('Time (s)')
    plt.ylabel('Wheel $\omega$')
    plt.legend(['Left wheel', 'Right wheel'])
    plt.grid()
    
    
    display.clear_output(wait=True)
    display.display(plt.gcf())
    time.sleep(0.1)