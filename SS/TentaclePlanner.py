import numpy as np
# from matplotlib import pyplot as plt
# from IPython import display

class TentaclePlanner:   
    def __init__(self,dt=0.1,steps=5,alpha=1,beta=0.1):
        
        self.dt = dt 
        self.steps = steps
        # Tentacles are possible trajectories to follow
        # (v, w) - linear velocity (forward speed), angular velocity (turning rate)
        self.tentacles = [(0.0,0.5),(0.0,-0.5),(0.1,1.0),(0.1,-1.0),(0.1,0.5),(0.1,-0.5),(0.1,0.0),(0.0,0.0)]
        
        # alpha and beta are weighting factors for cost function

        self.alpha = alpha # weighting factor for reaching goal position
        self.beta = beta # weighting factor for reaching goal orientation
    
    # Play a trajectory and evaluate where you'd end up
    def roll_out(self,v,w,goal_x,goal_y,goal_th,x,y,th):
        """
        """
        
        for j in range(self.steps):
        
            x = x + self.dt*v*np.cos(th)
            y = y + self.dt*v*np.sin(th)
            th = (th + w*self.dt)
        
        e_th = goal_th-th
        e_th = np.arctan2(np.sin(e_th),np.cos(e_th))
        
        return self.alpha*((goal_x-x)**2 + (goal_y-y)**2) + self.beta*(e_th**2)
    
    # Choose trajectory that will get you closest to the goal
    def plan(self,goal_x,goal_y,goal_th,x,y,th):
        
        costs =[]
        for v,w in self.tentacles:
            costs.append(self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th))
        
        best_idx = np.argmin(costs)
        
        return self.tentacles[best_idx]
        