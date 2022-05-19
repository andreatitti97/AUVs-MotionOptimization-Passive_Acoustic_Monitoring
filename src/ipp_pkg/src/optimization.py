#Import basic system modules
import os
import time
# Import math modules
import numpy as np
from math import sin, cos, pi
# Import ROS modules and Service
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
# Import costum classes
import importlib.util
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.tracker_optimization", class_path+"/tracker.py")
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)
spec = importlib.util.spec_from_file_location("module.sensor", class_path+"/sensor.py")
sensor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor)
# IMPORT GLOBAL VARIABLES FOR SIMULATION
spec = importlib.util.spec_from_file_location("module.main2", "/home/andrea/ros_simulation_ws/src/ipp_pkg/src/main2.py")
main2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(main2)
TIME_SCALER = main2.TIME_SCALER
TIME_STEP = main2.TIME_STEP
MEAS_VARIANCE = main2.MEAS_VARIANCE
TARGET_INIT = main2.TARGET_INIT
tc = main2.OPTIMIZATION_TIME_STEP
# FOLDER PATH DEFINITION
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/utils')
plot_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')

#GLOBAL VARIABLES - simulation parameters
key1 = -pi/6
key2 = 0
key3 = pi/6
keys = [key1, key2, key3]
key_final = None
target_theta = TARGET_INIT[2]
velTarget = TARGET_INIT[3]
# Sensors
sensor1 = sensor.Sensor('first_streamer',1,0,0,1)
sensor2 = sensor.Sensor('seconda_streamer',1,0,0,-1)
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

def simulation(control_input, target_init, platform_init, tracker):
    platform = Platform(platform_init)
    target = Target(target_init)
    platform_state = platform.update_state(control_input)  
    target_state = target.update_state() 

    #update measurament
    sensor1.vehiclePose(platform_state[0], platform_state[1], platform_state[2], tc)  
    sensor1.targetPoseReal(target_state[0], target_state[1], target_theta)
    
    sensor2.vehiclePose(platform_state[0], platform_state[1], platform_state[2], tc)
    sensor2.targetPoseReal(target_state[0], target_state[1], target_theta)
    
    [measure1,sensor_pose1] = sensor1.measureBearing()
    [measure2,sensor_pose2] = sensor2.measureBearing()
    measures = [measure1, measure2]
    # update EKF
    
    tracker.processMeasurement(measures,target_state, sensor_pose1, sensor_pose2, tc)
    [state, P] = tracker.state
    
    state = [state[0,0], state[1,0], state[2,0], state[3,0]]
    return state, P, platform_state, target_state

def compute_cost(P):
   
    cost = np.trace(P)
    return cost

def main():

    global covariance, target_est, platform_state
    # Sensor Initialization
    rospy.init_node('optimization')
    pub = rospy.Publisher("ctrl_cmd",numpy_msg(Floats),queue_size=100)
    Hz = 1/(TIME_STEP)
    rate = rospy.Rate(Hz)

    # Init plannin horizon, cost, ctrl_cmds, covarianc
    T = 4  
    cost = 0
    ctrl_cmd = []
    target_traj_est_x = []
    target_traj_est_y = []
    target_traj_real_x = []
    target_traj_real_y = []
    ctrl_plot = []
    P = np.eye((4))
    rospy.loginfo('STARTED OPTIMIZATION')
    while not rospy.is_shutdown():
        # INIT TARGET MODEL AND PLATFORM MODEL WITH THE LATEST ESTIMATION AND SENSOR POSITIONS 

        t_est = rospy.wait_for_message('/estimation',numpy_msg(Floats))
        s_state = rospy.wait_for_message('/platform_state',numpy_msg(Floats))
        covariance = rospy.wait_for_message('/covariance',numpy_msg(Floats))
        t_est = t_est.data
        s_state = s_state.data
        covariance = covariance.data
        for i in range(4):
                for j in range(4):
                    P[i,j] = covariance.data[i+j]

        tracker1 = tracker.Tracker('1', True, P)
        tracker2 = tracker.Tracker('2', True, P)
        tracker3 = tracker.Tracker('3', True, P)

        start = time.time()
        for t in range(T):
            for k in range(3):
                if k == 0:  
                    x1, P1, s1, x_real1 = simulation(keys[k], t_est, s_state, tracker1)
                    cost1 = compute_cost(P1)
                    #print('s1',x1)

                if k == 1:
                    x2, P2, s2, x_real2 = simulation(keys[k], t_est, s_state, tracker2)
                    cost2 = compute_cost(P2)
                    #print('s2',x2)
                if k == 2:
                    x3, P3, s3, x_real3 = simulation(keys[k], t_est, s_state, tracker3)
                    cost3 = compute_cost(P3)
                    #print('s3',x3)

            cost = cost1
            t_est = x1
            s_state = s1
            P = P1
            key_final = key1
            target_prediction = x_real1
            if cost2 < cost:
                key_final = key2
                cost = cost2
                t_est = x2
                s_state = s2
                P = P2
                target_prediction = x_real2
            if cost3 < cost:
                key_final = key3
                cost = cost3
                t_est = x3
                s_state = s3
                P = P3
                target_prediction = x_real3
            
            target_traj_est_x.append(t_est[0])
            target_traj_est_y.append(t_est[1])
            target_traj_real_x.append(target_prediction[0])
            target_traj_real_y.append(target_prediction[1])
            ctrl_cmd.append(key_final)
            ctrl_plot.append(key_final)

        rospy.sleep(TIME_STEP*10)
        pub.publish(np.array(ctrl_cmd,dtype=np.float32))

        #SAVE FILE FOR PLOT    
        np.savetxt(lib_path+'/ctrl_cmd.txt',ctrl_cmd)
        np.savetxt(plot_path+'/target_traj_est_x.txt',target_traj_est_x)
        np.savetxt(plot_path+'/target_traj_est_y.txt',target_traj_est_y)
        np.savetxt(plot_path+'/target_traj_real_x.txt',target_traj_real_x)
        np.savetxt(plot_path+'/target_traj_real_y.txt',target_traj_real_y)
        np.savetxt(plot_path+'/plot_cmds.txt',ctrl_plot)

        stop = time.time()
        #rospy.loginfo('OPTIMIZATION TIME:',(stop - start))
        rospy.loginfo(ctrl_cmd)
        ctrl_cmd = []
        rate.sleep()
 
if __name__ == '__main__':
    
    main()
