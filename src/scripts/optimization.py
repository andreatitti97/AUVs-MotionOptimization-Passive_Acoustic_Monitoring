#Import basic system modules
import sys
import os
import time
# Import math modules
import numpy as np
from math import sin, cos, pi
import numpy.matlib
# Import ROS modules
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
# Import costum classes
from Classes.sensor import Sensor
from Classes.tracker_optimization import Tracker
# PATH DEFINITION
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs/utils')
sys.path.append(lib_path)

#GLOBAL VARIABLES
count = 0
key1 = -pi/6
key2 = 0
key3 = pi/6
keys = [key1, key2, key3]
key_final = None
tc = 2
target_theta = pi/2
# Sensors
f1 = 1 #Hz
f2 = 1 #Hz
mean1 = 0
variance1 = 0.1
mean2 = 0
variance2 = 0.1
sensor1 = Sensor('first_streamer',f1,mean1,variance1,1)
sensor2 = Sensor('seconda_streamer',f2,mean2,variance2,-1)
    
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
        self.vlx = 1*cos(target_theta)#TODO make dynamic
        print(self.vlx)
        self.vly = 1*sin(target_theta)
        self.x = self.x + self.vlx*self.dt
        self.y = self.y + self.vly*self.dt
        return [self.x, self.y, self.vlx, self.vly]

class Node:
 
    # Constructor to create a new node
    def __init__(self, key, cost):

        self.key = key 
        self.level = 0

        self.left = None
        self.middle = None
        self.right = None
        self.cost = cost
        self.t_realization = None
        self.P_realization = None
        self.s_realization = None


def simulation(control_input, target_init, platform_init, init_cov, tracker1):
    platform = Platform(platform_init)
    target = Target(target_init)
    platform_state = platform.update_state(control_input)  
    target_state = target.update_state() 
    print(target_state)
    #update measurament
    sensor1.vehiclePose(platform_state[0], platform_state[1], platform_state[2])  
    sensor1.targetPoseNoisy(target_state[0], target_state[1], target_theta)
    
    sensor2.vehiclePose(platform_state[0], platform_state[1], platform_state[2])
    sensor2.targetPoseNoisy(target_state[0], target_state[1], target_theta)
    
    [measure1,sensor_pose1] = sensor1.measureBearing()
    [measure2,sensor_pose2] = sensor2.measureBearing()
    measures = [measure1, measure2]
    # update EKF
    
    tracker1.processMeasurement(measures,target_state, sensor_pose1, sensor_pose2, tc)
    [state, P] = tracker1.state
    
    state = [state[0,0], state[1,0], state[2,0], state[3,0]]
    return state, P, platform_state

def compute_cost(input_cost, P):
   
    cost = np.trace(P)
    #cost = input_cost + cost
    return cost

def minValueNode( node):
    current = node
 
    # loop down to find the leftmost leaf
    while(current.left is not None):
        current = current.left 
 
    return current 
        
 
# A utility function to do inorder traversal of TST
def inorder(root):
    if root is not None:

        inorder(root.left)
        print(root.cost)
        inorder(root.middle)
        inorder(root.right)
        
# A utility function to insert a new node with given key in TST
def insert1( node, key, cost):
    # If the tree is empty, return a new node
    global key_final
    if node is None:
        return Node(key, cost) #TODO
    
    if key == key1:
        node.left = insert1(node.left, key1, cost)
        
        node.level += 1
    if key == key2:
        node.middle = insert1(node.middle, key2, cost)
        node.level += 1
    if key == key3:
        node.right = insert1(node.right, key3, cost)
        node.level += 1
    #print(node.right.level)
    return node


def init_tree(node,key,init_cost=0):
    if node is None:
        return Node(key, init_cost)

def callback1(data):

    tmp = data.data
    
    target_est = [tmp[0], tmp[1], tmp[2], tmp[3]]
    print(data)
    np.savetxt(lib_path+'/target_est.txt',np.array(target_est,dtype=np.float32))


def callback2(data):

    tmp = data.data
    
    platform_state = [tmp[0], tmp[1], tmp[2]]
    
    np.savetxt(lib_path+'/platform_state.txt',np.array(platform_state,dtype=np.float32))

    
def callback3(data):
    P = data.data
    
    np.savetxt(lib_path+'/covariance.txt',np.array(P,dtype=np.float32))

def main():
    # Sensor Initialization
    rospy.init_node('optimization')
    pub_ctrl_cmd = rospy.Publisher('ctrl_cmd', numpy_msg(Floats), queue_size=100)
    rospy.Subscriber("estimation", numpy_msg(Floats),callback1)
    rospy.Subscriber("platform_state", numpy_msg(Floats),callback2)
    rospy.Subscriber("covariance", numpy_msg(Floats),callback3)
    rate = rospy.Rate(100) #loop spin at 100 Hz

    # Define Input for tree generation: tree level, estimated state


    T = 4  


    root = None
    cost = 0
    ctrl_cmd = []
    P_init = np.matlib.zeros((4,4))
    input('PRESS INVIO TO START OPTIMIZATION')
    print('start optimization')
    while not rospy.is_shutdown():
        # INIT TARGET MODEL AND PLATFORM MODEL WITH THE LATEST ESTIMATION AND SENSOR POSITIONS 
        
        target_est = np.loadtxt(lib_path+'/target_est.txt')
        target_est = [target_est[0],target_est[1],target_est[2],target_est[3]]
        platform_state = np.loadtxt(lib_path+'/platform_state.txt')
        platform_state = [platform_state[0],platform_state[1],platform_state[2]] 
        covariance = np.loadtxt(lib_path+'/covariance.txt')
        tracker1 = Tracker('first_observer', P_init)
        tracker2 = Tracker('second', P_init)
        tracker3 = Tracker('third', P_init)
        for i in range(4):
            for j in range(4):
                P_init[i,j] = covariance[i+j]

        costs1 = []
        costs2 = []
        costs3 = []
        t_est1 = [0.01, 0.01, 0, 0]
 
        s1 = [5, 1, pi/4]

        P1 = P_init


        root = init_tree(root, 0.001)
        
        start = time.time()
        for t in range(T):
            for k in range(3):
                
                
                
                if k == 0:
                    
                    xl1, Pl1, sl1 = simulation(keys[k], t_est1, s1, P1, tracker1)
                    cost1 = compute_cost(0, Pl1)
                    #
                    # print('s1',xl1)
                    costs1.append(cost1)

                        
                if k == 1:
                    xl2, Pl2, sl2 = simulation(keys[k], t_est1, s1, P1, tracker2)
                    cost2 = compute_cost(0, Pl2)
                    #print('s2',xl2)
                    costs2.append(cost2)
                if k == 2:
                    xl3, Pl3, sl3 = simulation(keys[k], t_est1, s1, P1, tracker3)
                    cost3 = compute_cost(0, Pl3)
                    #print('s3',xl3)
                    costs3.append(cost3)
            

            cost = cost1
            t_est1 = xl1
            s1 = sl1
                    
            P1 = Pl1
            if cost2 < cost:

                key_final = key2
                cost = cost2
                t_est1 = xl2
                s1 = sl2
                    
                P1 = Pl2
            if cost3 < cost:

                key_final = key3
                t_est1 = xl3
                s1 = sl3
                P1 = Pl3
            else:
                key_final = key1
            #root = insert1(root, key, cost)
            #tracker1 = Tracker('first_observer', P1)
            ctrl_cmd.append(key_final)

        print("Inorder traversal of the given tree")
        stop = time.time()
        print('OPTIMIZATION TIME:',stop - start)
        print(ctrl_cmd)
        pub_ctrl_cmd.publish(np.array(ctrl_cmd,np.float32))
        ctrl_cmd = []
        
        input('PRESS INVIO TO CONTINUE OPTIMIZATION')
        #root = None
        
if __name__ == '__main__':
    
    main()
