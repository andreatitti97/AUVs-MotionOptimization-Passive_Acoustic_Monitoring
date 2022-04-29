from curses.ascii import ctrl
from platform import platform
from sys import call_tracing
import cvxpy as cp
from matplotlib.colors import to_rgb
import numpy as np
import matplotlib.pyplot as plt
from Classes.sensor import Sensor
import numpy.matlib
from Classes.tracker_optimization import Tracker
from math import sin, cos, pi
import time
import rospkg
import rospy
import roslib
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String

count = 0
key1 = -pi/4
key2 = 0
key3 = pi/4
key_final = None
tc = 2
target_theta = pi/2
# Sensors
f1 = 1 #Hz
f2 = 1 #Hz
mean1 = 0
variance1 = 0
mean2 = 0
variance2 = 0
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

        self.dt += tc
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

        self.dt += 1
        self.vlx = 1*cos(target_theta)#TODO make dynamic
        self.vly = 1*sin(target_theta)
        self.x = self.x + self.vlx*self.dt
        self.y = self.x + self.vly*self.dt
        return [self.x, self.y, self.vlx, self.vly]

class Node:
 
    # Constructor to create a new node
    def __init__(self, key, cost):

        self.key = key 
        self.left = None
        self.middle = None
        self.right = None
        self.cost = cost
        self.t_realization = None
        self.P_realization = None
        self.s_realization = None


def compute_cost(input_cost, control_input, target_init, platform_init, init_cov, tracker1):
   
    
    platform = Platform(platform_init)
    target = Target(target_init)
    platform_state = platform.update_state(control_input)  
    target_state = target.update_state() 
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
    #print(state)
    cost = np.trace(P)
    cost = input_cost + cost
    return cost, state, P, platform_state

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
        print(root.P_realization)
        inorder(root.middle)
        inorder(root.right)
        
# A utility function to insert a new node with given key in TST
def insert1( node,key, cost, target_est, platform_state, covariance, tracker1):
    # If the tree is empty, return a new node
    global key_final
    if node is None:
        return Node(key, cost)

    new_cost1, t1, P1, s1 = compute_cost(node.cost,key1,target_est,platform_state, covariance, tracker1)
    #print(t1)
    #print(P1)
    #print(s1)
    tmpCost1 = new_cost1
    key_final = key1
    new_cost2, t2, P2, s2 = compute_cost(node.cost,key2,target_est,platform_state, covariance, tracker1)
    tmpCost2 = new_cost2
    new_cost3, t3, P3, s3 = compute_cost(node.cost,key3,target_est,platform_state, covariance, tracker1)
    tmpCost3 = new_cost3
    if tmpCost2 < tmpCost1:
        key_final = key2
        tmpCost1 = tmpCost2
    if tmpCost3 < tmpCost1:
        key_final = key3
        tmpCost2 = tmpCost3
    
    
    node.left = insert1(node.left, key1, new_cost1, t1, s1, P1, tracker1)
    node.left.cost = new_cost1
    node.left.t_realization = t1
    node.left.s_realization = s1
    node.left.P_realization = P1
    node.middle = insert1(node.middle, key2, new_cost2, t2, s2, P2, tracker1)
    node.middle.cost = new_cost2
    node.middle.t_realization = t2
    node.middle.s_realization = s2
    node.middle.P_realization = P2
    node.right = insert1(node.right, key3, new_cost3, t3, s3, P3, tracker1)
    node.right.cost = new_cost3
    node.right.t_realization = t3
    node.right.s_realization = s3
    node.right.P_realization = P3

    return node

def callback1(data):

    tmp = data.data
    
    target_est = [tmp[0], tmp[1], tmp[2], tmp[3]]
    
    np.savetxt('scripts/target_est.txt',np.array(target_est,dtype=np.float32))


def callback2(data):

    tmp = data.data
    
    platform_state = [tmp[0], tmp[1], tmp[2]]
    
    np.savetxt('scripts/platform_state.txt',np.array(platform_state,dtype=np.float32))

    
def callback3(data):
    P = data.data
    
    np.savetxt('scripts/covariance.txt',np.array(P,dtype=np.float32))

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
    time.sleep(5)
    print('start optimization')
    while not rospy.is_shutdown():
        # INIT TARGET MODEL AND PLATFORM MODEL WITH THE LATEST ESTIMATION AND SENSOR POSITIONS 
        
        target_est = np.loadtxt('scripts/target_est.txt')
        target_est = [target_est[0],target_est[1],target_est[2],target_est[3]]
        platform_state = np.loadtxt('scripts/platform_state.txt')
        platform_state = [platform_state[0],platform_state[1],platform_state[2]] 
        covariance = np.loadtxt('scripts/covariance.txt')
        tracker1 = Tracker('first_observer', P_init)
        for i in range(4):
            for j in range(4):
                P_init[i,j] = covariance[i+j]

        root = insert1(root, 0.001, cost, target_est, platform_state, P_init, tracker1)
        
        start = time.time()
        for t in range(T):
            
            root = insert1(root, 0, root.cost, target_est, platform_state, P_init, tracker1)
            ctrl_cmd.append(key_final)

        print("Inorder traversal of the given tree")
        inorder(root)
        #node = minValueNode(root)
        #print(node.key)
        stop = time.time()
        print('OPTIMIZATION TIME:',stop - start)
        print(ctrl_cmd)
        pub_ctrl_cmd.publish(np.array(ctrl_cmd,np.float32))
        ctrl_cmd = []
        
        time.sleep(60) # TIME BETWEEN OPTIMIZATION
        root = None
        
if __name__ == '__main__':
    
    main()


'''def deleteNode(root, target, tracker1, platform, sensor1, sensor2):
 
    # Base Case
    if root is None:
        return root 
 
    root.compute_cost(root.cost, key1, target, tracker1, platform, sensor1, sensor2)
    tmpCost = root.cost
    key_final = key1

    root.compute_cost(root.cost, key2, target, tracker1, platform, sensor1, sensor2) 
    tmpCost2 = root.cost
    if tmpCost2 <= tmpCost:
        root.left = deleteNode(root.left,target, tracker1, platform, sensor1, sensor2)
    else:
        root.middle = deleteNode(root.middle,target, tracker1, platform, sensor1, sensor2)
 
    root.compute_cost(root.cost, key3, target, tracker1, platform, sensor1, sensor2) 
    tmpCost3 = root.cost

    if tmpCost3 <= tmpCost:
        root.middle = deleteNode(root.middle, target, tracker1, platform, sensor1, sensor2)
    else:
        root.right = deleteNode(root.right, target, tracker1, platform, sensor1, sensor2)
    if root.left is None:
        tmp = root.middle
        root = None
        return tmp
    return root
'''