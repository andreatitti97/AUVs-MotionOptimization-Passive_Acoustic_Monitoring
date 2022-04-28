from platform import platform
from sys import call_tracing
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
from Classes.sensor import Sensor

from Classes.tracker import Tracker
from math import sin, cos, pi
import time
import rospkg
import rospy
import roslib
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String

count = 0
key1 = -pi/3
key2 = 0
key3 = pi/3

class Platform():
    def __init__(self, init_vector):
        self.x = init_vector[0]
        self.y = init_vector[1]
        self.theta = init_vector[2]
        self.vl = 1
        self.dt = 0
    def update_state(self, delta):

        self.dt += 0.01
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

        self.dt += 0.01
        self.vlx = 1*cos(pi/4)#TODO make dynamic
        self.vly = 1*sin(pi/4)
        self.x = self.x + self.vlx*self.dt
        self.y = self.x + self.vly*self.dt
        return [self.x, self.y, self.vlx, self.vly]

class Node:
 
    # Constructor to create a new node
    def __init__(self, key, cost, target_init, tracker_init, platform_init, sensor_init):
        
        self.level = None #level none defined
        self.key = key 
        self.left = None
        self.middle = None
        self.right = None
        self.target  = target_init
        self.tracker = tracker_init
        self.platform = platform_init
        self.sensor = sensor_init
        self.cost = cost

    def compute_cost(self, input_cost, control_input, target, tracker1, platform, sensor1):
        
        [xs, ys, thetas] = platform.update_state(control_input)  
        [xt, yt, xvt, yvt] = target.update_state()       
        sensor1.vehiclePose(xs, ys, thetas)
        #update measurament
        sensor1.targetPoseNoisy(xt, yt, pi/4)
        [measure1,sensor_pose] = sensor1.measureBearing()
        # update EKF
        tracker1.processMeasurement(measure1,[xt, yt, xvt, yvt], [xs, ys, thetas], 0.01)
        [state, P] = tracker1.state
        cost = np.trace(P)
        #print(cost)
        self.cost = input_cost + cost

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
        print(root.key)
        inorder(root.middle)
        inorder(root.right)
        
# A utility function to insert a new node with given key in TST

def insert1( node, key, cost, target, tracker1, platform, sensor1):
 
    # If the tree is empty, return a new node
    if node is None:
        print('nonevalue')
        return Node(key, cost, target, tracker1, platform, sensor1)
 
    # Otherwise recur down the tree
    if key == key1:
        
        node.compute_cost(cost, key1, target, tracker1, platform, sensor1)
        node.left = insert(node.left, key1, cost, target, tracker1, platform, sensor1)
        
        
        #print('check1',key)
    elif key == key2:
        node.compute_cost(cost, key2, target, tracker1, platform, sensor1)
        node.middle = insert(node.middle, key2, cost, target, tracker1, platform, sensor1)
       
        #print('check2',key)
    else:
        node.compute_cost(cost, key3, target, tracker1, platform, sensor1)
        node.right = insert(node.right, key3, cost, target, tracker1, platform, sensor1)
        #print('check3',key)
    return node

def insert( node, key, cost, target, tracker1, platform, sensor1):
 
    # If the tree is empty, return a new node
    if node is None:
      return Node(key, cost, target, tracker1, platform, sensor1)
 
    # Otherwise recur down the tree
    #if key == key1:
    node.compute_cost(cost, key, target, tracker1, platform, sensor1)
    tmpCost = node.cost
    '''key = key1
    node.compute_cost(cost, key2, target, tracker1, platform, sensor1)
    tmpCost2 = node.cost
    if tmpCost2 < tmpCost:
        key = key2
    node.compute_cost(cost, key3, target, tracker1, platform, sensor1)
    tmpCost3 = node.cost
    if tmpCost3 < tmpCost:
        key = key3'''

    if key == key1:
        node.left = insert(node.left, key1, cost, target, tracker1, platform, sensor1)

    elif key == key2:
        node.middle = insert(node.middle, key2, cost, target, tracker1, platform, sensor1)
    else:
        node.right = insert(node.right, key3, cost, target, tracker1, platform, sensor1)
    #print('check4',key)
    #time.sleep(1000)
        
    '''elif key == key2:
        node.compute_cost(cost, key1, target, tracker1, platform, sensor1)
        tmpCost = node.cost
        key = key1
        node.compute_cost(cost, key2, target, tracker1, platform, sensor1)
        tmpCost2 = node.cost
        if tmpCost2 < tmpCost:
            key = key2
        node.compute_cost(cost, key3, target, tracker1, platform, sensor1)
        tmpCost3 = node.cost
        if tmpCost3 < tmpCost:
            key = key3
        #node.compute_cost(cost, key1, target, tracker1, platform, sensor1)
        if key == key1:
            node.left = insert(node.left, key1, cost, target, tracker1, platform, sensor1)

        elif key == key2:
            node.middle = insert(node.middle, key2, cost, target, tracker1, platform, sensor1)
        else:
            node.right = insert(node.right, key3, cost, target, tracker1, platform, sensor1)
        #print('check5',key)
        #time.sleep(1000)
    else:
        node.compute_cost(cost, key1, target, tracker1, platform, sensor1)
        tmpCost = node.cost
        key = key1
        node.compute_cost(cost, key2, target, tracker1, platform, sensor1)
        tmpCost2 = node.cost
        if tmpCost2 < tmpCost:
            key = key2
        node.compute_cost(cost, key3, target, tracker1, platform, sensor1)
        tmpCost3 = node.cost
        if tmpCost3 < tmpCost:
            key = key3
        #node.compute_cost(cost, key1, target, tracker1, platform, sensor1)
        if key == key1:
            node.left = insert(node.left, key1, cost, target, tracker1, platform, sensor1)

        elif key == key2:
            node.middle = insert(node.middle, key2, cost, target, tracker1, platform, sensor1)
        else:
            node.right = insert(node.right, key3, cost, target, tracker1, platform, sensor1)
        #print('check6',key)'''
    # return the (unchanged) node pointer
    return node

def init_tree(node, key, cost, target, tracker1, platform, sensor1):
    if node is None:

        return Node(key, cost, target, tracker1, platform, sensor1)

def callback1(data):

    tmp = data.data
    
    target_est = [tmp[0], tmp[1], tmp[2], tmp[3]]
    
    np.savetxt('scripts/target_est.txt',np.array(target_est,dtype=np.float32))
    #print(rospy.get_name(), "I heard %s"%str(data.data))

def callback2(data):

    tmp = data.data
    
    platform_state = [tmp[0], tmp[1], tmp[2]]
    
    np.savetxt('scripts/platform_state.txt',np.array(platform_state,dtype=np.float32))
    #print(rospy.get_name(), "I heard %s"%str(data.data))
    
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
    # Sensors
    f1 = 1 #Hz
    f2 = 1 #Hz
    mean1 = 0
    variance1 = 0
    mean2 = 0
    variance2 = 0
    sensor1 = Sensor('first_streamer',f1,mean1,variance1,1)
    #sensor2 = Sensor('seconda_streamer',f2,mean2,variance2,-1)
    tracker1 = Tracker('first_observer')
    #tracker2 = Tracker('second_observer')

    # Define Input for tree generation: tree level, estimated state


    T = 3


    root = None
    cost = 0
    

    while not rospy.is_shutdown():
        # INIT TARGET MODEL AND PLATFORM MODEL WITH THE LATEST ESTIMATION AND SENSOR POSITIONS 
        
        target_est = np.loadtxt('scripts/target_est.txt')
        target_est = [target_est[0],target_est[1],target_est[2],target_est[3]]
        target = Target(target_est)
        platform_state = np.loadtxt('scripts/platform_state.txt')
        platform_state = [platform_state[0],platform_state[1],platform_state[2]] 
        platform = Platform(platform_state)
        covariance = np.loadtxt('scripts/covariance.txt')

        root = insert(root, 0.001, cost, target, tracker1, platform, sensor1)

        start = time.time()
        for t in range(T):
            #root = bnb(root, root.cost, target, tracker1, platform, sensor1)
            
            root = insert(root, key1, root.cost, target, tracker1, platform, sensor1)
            #print('provas')
            root = insert(root, key2, root.cost, target, tracker1, platform, sensor1)
            root = insert(root, key3, root.cost, target, tracker1, platform, sensor1)

        print("Inorder traversal of the given tree")
        inorder(root)
        #node = minValueNode(root)
        #print(node.cost)
        stop = time.time()
        print('OPTIMIZATION TIME:',stop - start)
        time.sleep(1000) # SHOULD WAIT UNTIL NEXT OPTIMIZATION REQUIRED
        rospy.spin()
        rate.sleep()
       
if __name__ == '__main__':
    
    main()





''' if key == key1:
node.compute_cost(cost, key, target, tracker1, platform, sensor1)
tmpCost = node.cost
node.left = insert(node.left, key, cost, target, tracker1, platform, sensor1)


elif key == key2:
node.compute_cost(cost, key2, target, tracker1, platform, sensor1)
tmpCost = node.cost
node.middle = insert(node.middle, key, cost, target, tracker1, platform, sensor1)


else:
node.compute_cost(cost, key3, target, tracker1, platform, sensor1)
tmpCost = node.cost
node.right = insert(node.right, key, cost, target, tracker1, platform, sensor1)'''


'''def insert( node, key, cost, target, tracker1, platform, sensor1):
 
    # If the tree is empty, return a new node
    if node is None:
        return Node(key, cost, target, tracker1, platform, sensor1)
 
    # Otherwise recur down the tree
    if key == key1:
        node.compute_cost(cost, key1, target, tracker1, platform, sensor1)
        node.left = insert(node.left, key1, cost, target, tracker1, platform, sensor1)

        node.compute_cost(cost, key2, target, tracker1, platform, sensor1)
        node.middle = insert(node.middle, key2, cost, target, tracker1, platform, sensor1)

        node.compute_cost(cost, key3, target, tracker1, platform, sensor1)
        node.right = insert(node.right, key3, cost, target, tracker1, platform, sensor1)
      
        
    elif key == key2:
        node.compute_cost(cost, key1, target, tracker1, platform, sensor1)
        node.left = insert(node.left, key1, cost, target, tracker1, platform, sensor1)

        node.compute_cost(cost, key2, target, tracker1, platform, sensor1)
        node.middle = insert(node.middle, key2, cost, target, tracker1, platform, sensor1)

        node.compute_cost(cost, key3, target, tracker1, platform, sensor1)
        node.right = insert(node.right, key3, cost, target, tracker1, platform, sensor1)
 
    else:
        node.compute_cost(cost, key1, target, tracker1, platform, sensor1)
        node.left = insert(node.left, key1, cost, target, tracker1, platform, sensor1)

        node.compute_cost(cost, key2, target, tracker1, platform, sensor1)
        node.middle = insert(node.middle, key2, cost, target, tracker1, platform, sensor1)

        node.compute_cost(cost, key3, target, tracker1, platform, sensor1)
        node.right = insert(node.right, key3, cost, target, tracker1, platform, sensor1)
    
    # return the (unchanged) node pointer
    return node'''
