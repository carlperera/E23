import numpy as np


WHEEL_SEP = (147 + 64)/1000
WHEEL_RAD = (53/2)/1000


class DiffDriveRobot:
    def __init__(self,inertia=5, dt=0.1, drag=0.2, wheel_radius=WHEEL_RAD, wheel_sep=WHEEL_SEP, max_speed = 3, init_weight = 0.5):
        
        self.x = 0.0 # y-position
        self.y = 0.0 # y-position 
        self.th = 0.0 # orientation
        
        self.max_speed = max_speed
        self.wl = 0.0 #rotational velocity left wheel
        self.wr = 0.0 #rotational velocity right wheel
        
        self.I = inertia 
        self.d = drag
        self.dt = dt # time step for updates - default is 0.1 s
        
        self.r = wheel_radius
        self.l = wheel_sep
        self.weight = init_weight # initial weight (without any tennis balls)
    
    # # Should be replaced by motor encoder measurement which measures how fast wheel is turning
    def motor_simulator(self,w,duty_cycle):
        """
        Args:
            :w - current speed
            :duty cycle - the measured duty cycle?
        """
        
        torque = self.I*duty_cycle
        
        if (w > 0):
            w = min(w + self.dt*(torque - self.d*w),3)
        elif (w < 0):
            w = max(w + self.dt*(torque - self.d*w),-3)
        else:
            w = w + self.dt*(torque)
        
        return w
        
    # Veclocity motion model
    def base_velocity(self,wl,wr):
        
        v = (wl*self.r + wr*self.r)/2.0
        
        w = (wl*self.r - wr*self.r)/self.l
        
        return v, w
    
    def duty_cycle_to_velocity(self, duty_cycle):
        # You can adjust the conversion factor based on your motor specifications
        # For example, let's assume a simple linear relationship for demonstration
       
        return duty_cycle * self.max_speed
    
    # Kinematic motion model
    def pose_update(self,duty_cycle_l,duty_cycle_r):
        
        self.wl = self.duty_cycle_to_velocity(duty_cycle_l)
        self.wr = self.duty_cycle_to_velocity(duty_cycle_r)
        
        v, w = self.base_velocity(self.wl, self.wr)
        
        self.x += self.dt * v * np.cos(self.th)
        self.y += self.dt * v * np.sin(self.th)
        self.th += w * self.dt
        
        return self.x, self.y, self.th
