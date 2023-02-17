from math import atan2, pi
import numpy as np

class Sensor:
    """ Simulate Vector Sensor """
    
    def __init__(self, name, f, mean, variance):
        self.name = name
        self.f = f
        self.mean = mean
        self.variance = variance

    def measureBearing(self,xt,yt,xo,yo):
        vect = [xt[1]-xo[1],yt[0]-yo[0]]
        self.abs_bearing = atan2(vect[0],vect[1]) # abs bearing = rel_bearing - vehcile ori -> [-pi,+pi]
        
        if self.theta_v < 0:
            theta_tmp = 2*pi + self.theta_v
        else:
            theta_tmp = self.theta_v

        if self.abs_bearing < 0:

            self.abs_bearing = 2*pi + self.abs_bearing # change convention Bearing_abs -> [0,2*pi]
        
        if theta_tmp  <= self.abs_bearing: # if the target is counter clock wise w.r.t to surge vel
            rel_bearing = self.abs_bearing - theta_tmp 
        else:
            rel_bearing = 2*pi - (theta_tmp - self.abs_bearing)

        # Create the noise and add the noise to the measurament
        #self.noise = np.random.uniform(-self.variance, self.variance)
        self.noise = np.random.normal(0,self.variance)
        self.abs_bearing = self.abs_bearing + self.noise #overwrite absolute bearing with the corrupted quantities
        return self.abs_bearing, self.w_pose_s, rel_bearing

    