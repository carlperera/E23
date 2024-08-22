import numpy as np

WHEEL_SEP = (147 + 64)/1000
WHEEL_RAD = (53/2)/1000

# pass in the pin numbers for this robot 
class DiffDriveRobot:
    def __init__(self,inertia=5, dt=0.1, drag=0.2, wheel_radius=WHEEL_RAD, wheel_sep=WHEEL_SEP, max_speed = 3, init_weight = 0.5):
        
        self.reset_init_position(x=0.0,y=0.0,th=0.0)
        
        self.max_speed = max_speed
        self.wl = 0.0 #rotational velocity left wheel
        self.wr = 0.0 #rotational velocity right wheel
        
        self.I = inertia 
        self.d = drag
        self.dt = dt # time step for updates - default is 0.1 s
        
        self.rad = wheel_radius
        self.wheel_sep = wheel_sep
        self.weight = init_weight # initial weight (without any tennis balls)
        
    
    def reset_init_position(self, x, y, th):
        self.x = x
        self.y = y
        self.th = th 

    #  Converts radial velocity of left and right wheel into velocial and radial velocity of entire robot 
    def base_velocity(self,wl,wr):
        
        v = (wl*self.rad + wr*self.rad)/2.0
        
        w = (wl*self.rad- wr*self.rad)/self.wheel_sep
        
        return v, w
    
   
    
    # Internal state holder 
    def pose_update(self, rad_vel_left, rad_vel_right):
        # convert rpms into radial velocity for both left and right motors 
        self.wl = rad_vel_left # get the 
        self.wr = rad_vel_right
        
        v, w = self.base_velocity(self.wl, self.wr)
        
        self.x += self.dt * v * np.cos(self.th)
        self.y += self.dt * v * np.sin(self.th)
        self.th += w * self.dt
        
        return self.x, self.y, self.th
