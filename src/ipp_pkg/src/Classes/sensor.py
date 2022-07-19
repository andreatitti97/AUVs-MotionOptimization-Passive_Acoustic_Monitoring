from math import atan2, pi
import numpy as np

class Sensor:
    """ Simulate Streamer """
    
    def __init__(self, name, f, mean, variance, sign, baseline_x, baseline_y):
        self.name = name
        self.f = f
        self.mean = mean
        self.variance = variance
        d = baseline_y
        self.baseline = sign*d 
        self.baseline_x = -baseline_x
        self.lin_vel = 1
        self.w_pose_t = []

    def vehiclePose(self, x_v, y_v, theta_v):#w.r.t the <w> - return auv pose from refernce pose
        self.theta_v = theta_v
        x_v = x_v + self.baseline_x*np.cos(self.theta_v)
        y_v = y_v + self.baseline_x*np.sin(self.theta_v)
        self.w_pose_s = np.array([(x_v+np.sin(self.theta_v)*self.baseline/2)+np.cos(self.theta_v)*self.lin_vel,
                                    (y_v-np.cos(self.theta_v)*self.baseline/2)+np.sin(self.theta_v)*self.lin_vel,
                                        self.theta_v])
 
    def targetPoseReal(self, x_t, y_t, theta_t=0):#w.r.t. the  <w>
        self.w_pose_t = np.transpose([x_t, y_t, theta_t])

    def measureBearing(self):
        vect = [self.w_pose_t[1]-self.w_pose_s[1],self.w_pose_t[0]-self.w_pose_s[0]]
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
        #np.random.seed(10)
        self.noise = np.random.uniform(0, self.variance) #can be also normal
        self.abs_bearing = self.abs_bearing + self.noise #overwrite absolute bearing with the corrupted quantities
    
        return self.abs_bearing, self.w_pose_s, rel_bearing 

    