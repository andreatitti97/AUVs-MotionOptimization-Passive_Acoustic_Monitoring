
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
        d = 10
        self.baseline = sign*d 
        self.noise = np.random.normal(self.mean, self.variance)
        self.measure = []
        

    def vehiclePose(self, x_v, y_v, theta_v):
        self.theta_v = theta_v
        self.wTv = transformation_matrix(x_v, y_v, theta_v)

        v_pose_s = [0, self.baseline/2, 1]
        tmp = np.dot(self.wTv,v_pose_s)
        self.w_pose_s = np.array([tmp[0],tmp[1], theta_v])

    def targetPoseReal(self, x_t, y_t, theta_t):#w.r.t. the  <w>
        self.w_pose_t = np.transpose([x_t, y_t, theta_t])
        #to do target pose only linear positions and vel

    def targetPoseNoisy(self, x_t, y_t, theta_t):
        self.w_poseNoisy_t = np.transpose([x_t, y_t, theta_t])+self.noise
        self.wTt = transformation_matrix(self.w_poseNoisy_t[0], self.w_poseNoisy_t[1], self.w_poseNoisy_t[2])
        self.vTt = np.linalg.inv(self.wTv)*self.wTt
        self.v_poseNoisy_t = np.dot(self.vTt,self.w_poseNoisy_t)
        

    def measureBearing(self):

        vect = [self.w_poseNoisy_t[1]-self.w_pose_s[1],self.w_poseNoisy_t[0]-self.w_pose_s[0]]
        self.abs_bearing = atan2(vect[0],vect[1]) 
        return self.abs_bearing, self.w_pose_s

    