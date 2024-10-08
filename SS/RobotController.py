class RobotController:
    
    def __init__(self,Kp=0.1,Ki=0.01,wheel_radius=0.02, wheel_sep=0.1):
        
        self.Kp = Kp
        self.Ki = Ki
        self.r = wheel_radius
        self.l = wheel_sep
        self.e_sum_l = 0
        self.e_sum_r = 0
        
    def pi_control(self,w_desired,w_measured,e_sum):
        """Outputs the duty cycle required and the e_sum to get to your w_desired from your current w_measured
        """
        
        duty_cycle = min(max(-1,self.Kp*(w_desired-w_measured) + self.Ki*e_sum),1)
        
        e_sum = e_sum + (w_desired-w_measured)
        
        return duty_cycle, e_sum
        
        
    def drive(self,v_desired,w_desired,wl,wr):
        """
        takes in desired v and desirerd w, and the current radial speeds of left and right, and outputs the 
        duty cycles for both wheels to give desired speeds (to adjust)
        """
        
        wl_desired = (v_desired + self.l*w_desired/2)/self.r
        wr_desired = (v_desired - self.l*w_desired/2)/self.r
        
        duty_cycle_l,self.e_sum_l = self.pi_control(wl_desired,wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r = self.pi_control(wr_desired,wr,self.e_sum_r)
        
        return duty_cycle_l, duty_cycle_r