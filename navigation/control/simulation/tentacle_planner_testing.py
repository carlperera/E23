import numpy as np
from matplotlib import pyplot as plt
from IPython import display

from DiffDriveRobot import DiffDriveRobot
from RobotController import RobotController
from TentaclePlanner import TentaclePlanner

robot = DiffDriveRobot(inertia=10, dt=0.1, drag=2, wheel_radius=0.05, wheel_sep=0.15)
controller = RobotController(Kp=1.0,Ki=0.15,wheel_radius=0.05,wheel_sep=0.15)
planner = TentaclePlanner(dt=0.1,steps=5,alpha=1,beta=1e-5)



plt.figure(figsize=(15,9))

poses = []
velocities = []
duty_cycle_commands = []

goal_x = 2*np.random.rand()-1
goal_y = 2*np.random.rand()-1
goal_th = 2*np.pi*np.random.rand()-np.pi

for i in range(200):

    # Plan using tentacles
    v,w = planner.plan(goal_x,goal_y,goal_th,robot.x,robot.y,robot.th)
    
    duty_cycle_l,duty_cycle_r = controller.drive(v,w,robot.wl,robot.wr)
    
    # Simulate robot motion - send duty cycle command to robot
    x,y,th = robot.pose_update(duty_cycle_l,duty_cycle_r)
    
    # Log data
    poses.append([x,y,th])
    duty_cycle_commands.append([duty_cycle_l,duty_cycle_r])
    velocities.append([robot.wl,robot.wr])
    
    # Plot robot data
    plt.clf()
    plt.subplot(1,2,1)
    plt.plot(np.array(poses)[:,0],np.array(poses)[:,1])
    plt.plot(x,y,'k',marker='+')
    plt.quiver(x,y,0.1*np.cos(th),0.1*np.sin(th))
    plt.plot(goal_x,goal_y,'x',markersize=5)
    plt.quiver(goal_x,goal_y,0.1*np.cos(goal_th),0.1*np.sin(goal_th))
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
    