from math import atan2, pi
import numpy as np

def transformation_matrix(x, y, theta):
    return np.array([
    [np.cos(theta), -np.sin(theta), x],
    [np.sin(theta), np.cos(theta), y],
    [0, 0, 1]
])

class Sensor:
    """ Simulate Streamer """
    
    def __init__(self,name, f, mean, variance, sign):
        self.name = name
        self.f = f
        self.mean = mean
        self.variance = variance
        d = 200
        self.baseline = sign*d 
        self.w_pose_t = []

    def vehiclePose(self, x_v, y_v, theta_v, dt):#w.r.t the <w> - return sensor pose from vehicle pose
        

        self.theta_v = theta_v + dt
        v_pose_s = [0, self.baseline/2]
        '''self.w_pose_s = np.array([(x_v+v_pose_s[0])+np.cos(self.theta_v)*dt,
                                    (y_v+v_pose_s[1])+np.sin(self.theta_v)*dt,self.theta_v])'''
                                        
        self.w_pose_s = np.array([(x_v+np.sin(self.theta_v)*self.baseline/2)+np.cos(self.theta_v)*dt,
                                        (y_v-np.cos(self.theta_v)*self.baseline/2)+np.sin(self.theta_v)*dt, self.theta_v])
        '''tmp = atan2(self.w_pose_s[1],self.w_pose_s[0])
        if tmp < atan2(y_v,x_v):
            self.theta_v = theta_v
            self.w_pose_s = np.array([(x_v+np.sin(self.theta_v)*self.baseline/2)+np.cos(self.theta_v),
                                        (y_v-np.cos(self.theta_v)*self.baseline/2)+np.sin(self.theta_v), self.theta_v])'''

    def targetPoseReal(self, x_t, y_t, theta_t):#w.r.t. the  <w>
        self.w_pose_t = np.transpose([x_t, y_t, theta_t])

    def measureBearing(self):
        vect = [self.w_pose_t[1]-self.w_pose_s[1],self.w_pose_t[0]-self.w_pose_s[0]]
        self.abs_bearing = atan2(vect[0],vect[1]) # abs bearing = rel_bearing - vehcile ori -> [-pi,+pi]

        if self.abs_bearing < 0:

            self.abs_bearing = 2*pi + self.abs_bearing # change convention Bearing_abs -> [0,2*pi]
        if self.theta_v  <= self.abs_bearing: # if the target is counter clock wise w.r.t to surge vel
            rel_bearing = self.abs_bearing - self.theta_v 
        else:
            rel_bearing = 2*pi - self.theta_v - self.abs_bearing

        self.noise = np.random.normal(self.mean, self.variance)
        A = 10
        epsi = 1
        activation_function = A*np.cos(rel_bearing)
        #TODO - I THINK IS OK
        if activation_function < 0: 
            activation_function = activation_function-epsi
        else:
            activation_function = activation_function+epsi
        self.w_pose_t = [self.w_pose_t[0]+self.noise*activation_function, self.w_pose_t[1]+self.noise*activation_function,
                            self.w_pose_t[2]]
        vect =  [self.w_pose_t[1]-self.w_pose_s[1],self.w_pose_t[0]-self.w_pose_s[0]]
        self.abs_bearing = atan2(vect[0],vect[1])#overwrite absolute bearing with the corrupted quantities

        return self.abs_bearing, self.w_pose_s

    