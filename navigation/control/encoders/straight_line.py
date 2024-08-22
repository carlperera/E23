import numpy as np
import RPi.GPIO as GPIO
from RobotController import RobotController
from DiffDriveRobot import DiffDriveRobot
from TentaclePlanner import TentaclePlanner

WHEEL_SEP = (147 + 64)/1000
WHEEL_RAD = (53/2)/1000

robot = DiffDriveRobot(inertia=5, dt=0.1, drag=1, wheel_radius=WHEEL_RAD, wheel_sep=WHEEL_SEP)
controller = RobotController(Kp=1,Ki=0.25,wheel_radius=WHEEL_RAD,wheel_sep=WHEEL_SEP)
planner = TentaclePlanner(dt=0.1,steps=5,alpha=1,beta=1e-5)

w = 0
w_desired = 2.0
w_measured = 0.0
duty_cycle = 0

poses = []
velocities = []
duty_cycle_commands = []

# Plan using tentacles
v,w = planner.plan(0,0.1,0,robot.x,robot.y,robot.th)

duty_cycle_l,duty_cycle_r = controller.drive(v,w,robot.wl,robot.wr)

# Simulate robot motion - send duty cycle command to robot
x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r)

# Log data
poses.append([x,y,th])
duty_cycle_commands.append([duty_cycle_l,duty_cycle_r])
velocities.append([robot.wl,robot.wr])