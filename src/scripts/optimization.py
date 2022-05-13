#Import basic system modules
import sys
import os
import time
# Import math modules
import numpy as np
from math import sin, cos, pi
# Import ROS modules
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
# Import costum classes
from Classes.sensor import Sensor
from Classes.tracker_optimization import Tracker
from main2 import TIME_SCALER, TIME_STEP, TARGET_INIT, MEAS_VARIANCE
# PATH DEFINITION
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs/utils')
sys.path.append(lib_path)

#GLOBAL VARIABLES - simulation parameters
key1 = -pi/6
key2 = 0
key3 = pi/6
keys = [key1, key2, key3]
key_final = None
tc = 2 #elapsed time for EKF simulation (how much time we predict the target movement for each time step)
target_theta = TARGET_INIT[2]
velTarget = TARGET_INIT[3]
# Sensors
sensor1 = Sensor('first_streamer',1,0,MEAS_VARIANCE,1)
sensor2 = Sensor('seconda_streamer',1,0,MEAS_VARIANCE,-1)
# Init global variables for callbacks
platform_state = []
target_est = []
covariance = []

class Platform():
    def __init__(self, init_vector):
        self.x = init_vector[0]
        self.y = init_vector[1]
        self.theta = init_vector[2]
        self.vl = 1
        self.dt = 0
    def update_state(self, delta):

        self.dt = tc
        self.theta = self.theta + delta
        self.x = self.x + cos(self.theta)*self.vl*self.dt
        self.y = self.x + cos(self.theta)*self.vl*self.dt
        
        return [self.x, self.y, self.theta]

class Target():
    def __init__(self, init_vector):
        self.x = init_vector[0]
        self.y = init_vector[1]

        self.vlx = init_vector[2]
        self.vly = init_vector[3]
        self.dt = 0

    def update_state(self):

        self.dt = tc
        self.vlx = velTarget*cos(target_theta)
        self.vly = velTarget*sin(target_theta)
        self.x = self.x + self.vlx*self.dt
        self.y = self.y + self.vly*self.dt
        return [self.x, self.y, self.vlx, self.vly]

def simulation(control_input, target_init, platform_init, init_cov, tracker1):
    platform = Platform(platform_init)
    target = Target(target_init)
    platform_state = platform.update_state(control_input)  
    target_state = target.update_state() 

    #update measurament
    sensor1.vehiclePose(platform_state[0], platform_state[1], platform_state[2])  
    sensor1.targetPoseReal(target_state[0], target_state[1], target_theta)
    
    sensor2.vehiclePose(platform_state[0], platform_state[1], platform_state[2])
    sensor2.targetPoseReal(target_state[0], target_state[1], target_theta)
    
    [measure1,sensor_pose1] = sensor1.measureBearing()
    [measure2,sensor_pose2] = sensor2.measureBearing()
    measures = [measure1, measure2]
    # update EKF
    
    tracker1.processMeasurement(measures,target_state, sensor_pose1, sensor_pose2, tc)
    [state, P] = tracker1.state
    
    state = [state[0,0], state[1,0], state[2,0], state[3,0]]
    return state, P, platform_state

def compute_cost(P):
   
    cost = np.trace(P)
    return cost

def callback1(data):

    global target_est
    tmp = data.data
    target_est = [tmp[0], tmp[1], tmp[2], tmp[3]]

def callback2(data):

    global platform_state
    tmp = data.data
    platform_state = [tmp[0], tmp[1], tmp[2]]

def callback3(data):
    global covariance
    covariance = data.data

def main():

    global covariance, target_est, platform_state
    # Sensor Initialization
    rospy.init_node('optimization')
    rospy.Subscriber("estimation", numpy_msg(Floats),callback1)
    rospy.Subscriber("platform_state", numpy_msg(Floats),callback2)
    rospy.Subscriber("covariance", numpy_msg(Floats),callback3)
    Hz = 1/(TIME_STEP)
    rate = rospy.Rate(Hz)

    # Init plannin horizon, cost, ctrl_cmds, covarianc
    T = 4  
    cost = 0
    ctrl_cmd = []
    P = np.eye((4))
    input('PRESS INVIO TO START OPTIMIZATION')
    print('start optimization')
    time.sleep((T*2)/TIME_SCALER)
    while not rospy.is_shutdown():
        # INIT TARGET MODEL AND PLATFORM MODEL WITH THE LATEST ESTIMATION AND SENSOR POSITIONS 

        t_est = target_est
        s_state = platform_state
        for i in range(4):
                for j in range(4):
                    P[i,j] = covariance[i+j]

        tracker1 = Tracker('1', P)
        tracker2 = Tracker('2', P)
        tracker3 = Tracker('3', P)

        start = time.time()
        for t in range(T):
            for k in range(3):
                if k == 0:  
                    x1, P1, s1 = simulation(keys[k], t_est, s_state, P, tracker1)
                    cost1 = compute_cost(P1)
                    #print('s1',x1)

                if k == 1:
                    x2, P2, s2 = simulation(keys[k], t_est, s_state, P, tracker2)
                    cost2 = compute_cost(P2)
                    #print('s2',x2)
                if k == 2:
                    x3, P3, s3 = simulation(keys[k], t_est, s_state, P, tracker3)
                    cost3 = compute_cost(P3)
                    #print('s3',x3)

            cost = cost1
            t_est = x1
            s_state = s1
            P = P1
            key_final = key1
            if cost2 < cost:
                key_final = key2
                cost = cost2
                t_est = x2
                s_state = s2
                P = P2
            if cost3 < cost:
                key_final = key3
                cost = cost3
                t_est = x3
                s_state = s3
                P = P3
                
            ctrl_cmd.append(key_final)
        np.savetxt(lib_path+'/ctrl_cmd.txt',ctrl_cmd)
        stop = time.time()
        print('OPTIMIZATION TIME:',(stop - start))
        print(ctrl_cmd)
        ctrl_cmd = []
        rate.sleep()
        time.sleep((T*2)/TIME_SCALER)
        
        
if __name__ == '__main__':
    
    main()
