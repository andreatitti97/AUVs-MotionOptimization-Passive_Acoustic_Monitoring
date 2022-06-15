#
# This example script defines and solves
# a minimal pybnb example problem.
#
# It can be executed in serial as
#
# $ python simple.py
#
# or in parallel as
#
# $ mpiexec -n <n> python simple.py
#
# The mpi4py module is required.
#
from curses.ascii import ctrl
from random import choices
import pybnb
#Import basic system modules
import os
import time
# Import math modules
import numpy as np
from math import cos, pi, sin
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

target_theta = pi/4 #TARGET_INIT[2]
velTarget = 3 #TARGET_INIT[3]

# Sensors
sensor1 = sensor.Sensor('first_streamer',1,0,0,1)
sensor2 = sensor.Sensor('seconda_streamer',1,0,0,-1)
# Init global variables for callbacks
platform_state = []
target_est = []
# OPTIMIZATION PARAMETERS
key1 = -pi/12
key2 = 0
key3 = pi/12
key4 = -pi/15
key5 = + pi/15
DELTA = 10**9
ctrl_cmd = [key1, key2, key3, key5, key5] #BUG 
tc = 8

class Platform():
    def __init__(self, init_vector):
        self.x = init_vector[0]
        self.y = init_vector[1]
        self.theta = init_vector[2]
        self.vl = 1
        self.dt = tc
    def update_state(self, delta):

        self.theta = self.theta + delta
        self.x = self.x + cos(self.theta)*self.vl*self.dt
        self.y = self.y + sin(self.theta)*self.vl*self.dt
        
        return [self.x, self.y, self.theta]

class Target():
    def __init__(self, init_vector):
        self.x = init_vector[0]
        self.y = init_vector[1]
        
        self.vlx = init_vector[2]
        self.vly = init_vector[3]
        self.dt = tc

    def update_state(self):

        self.x = self.x + self.vlx*self.dt
        self.y = self.y + self.vly*self.dt
        return [self.x, self.y, self.vlx, self.vly]

def simulation(control_input, target_est, platform_pose, tracker):

    platform = Platform(platform_pose)
    target = Target(target_est)
    platform_state = platform.update_state(control_input)  
    target_state = target.update_state() 

    #update measurament
    sensor1.vehiclePose(platform_state[0], platform_state[1], platform_state[2], tc)  
    sensor1.targetPoseReal(target_state[0], target_state[1], target_theta)
    
    sensor2.vehiclePose(platform_state[0], platform_state[1], platform_state[2], tc)
    sensor2.targetPoseReal(target_state[0], target_state[1], target_theta)
    
    [measure1,sensor_pose1, rel_bearing1] = sensor1.measureBearing()
    [measure2,sensor_pose2, rel_bearing2] = sensor2.measureBearing()
    measures = [measure1, measure2]
    # update EKF
    
    tracker.processMeasurement(measures,target_state, sensor_pose1, sensor_pose2, tc)
    [state, P] = tracker.state
    state = [state[0,0], state[1,0], state[2,0], state[3,0]]
    return state, P, platform_state

def compute_cost(P):
   
    cost = np.trace(P)
    return cost

class Simple(pybnb.Problem):
    def __init__(self, x_hat, s, P, initial_cost, tracker1, tracker2, tracker3, tracker4, tracker5):
        # aggiungi un livello per imporre un orizzonte finito 
        self._x_hat = x_hat
        self._s = s
        self._P = P
        self.value = initial_cost #fake obj
        self._bound = 0 #lower bound 
        self._objective = 0 #real obj
        self.choices = []
        self.tracker1 = tracker1
        self.tracker2 = tracker2
        self.tracker3 = tracker3
        self.tracker4 = tracker4
        self.tracker5 = tracker5

    # required methods
    def sense(self):
        return pybnb.minimize

    def objective(self):#TODO: L'OBJECTIVE E VALUE DEL NODO CHE È IL COSTO ACCUMULATO + IL NUOVO COSTO (vedi esempio knapsnack)
        #assert self.value is not None
        print('obj',self.value)
        return self.value

    def bound(self): # il bound è esclusivamente sull objective - CORRISPONDE AL COSTO ACCUMULATO FINO AL NODO IN ESAME
        #TODO il bound è dato dal solo costo accumulato, devi quindi calcolarlare il nuovo costo e fare  eventuali check 
        bound = self._bound
        print('bound:',bound)
        return bound

    def save_state(self, node):
        node.state = (self._x_hat, self._s, self._P, self.value, self._bound, self.choices)

    def load_state(self, node):
        (self._x_hat, self._s, self._P, self.value, self._bound, self.choices) = node.state

    def branch(self): #durante il branch devi calcolare le varie realizzazioni quindi simuli qua
        # qui carica lo stato del nodo padre e genera 3 figli a cui assegnare i vari costi e stati, ricorda che devi far ereditare
        # anche le realizzazioni del trget e della piattaforma e P, non solo il costo.
        x_hat, s, P = self._x_hat, self._s, self._P
    
        x1, P1, s1 = simulation(ctrl_cmd[0], x_hat, s, self.tracker1)
        x2, P2, s2 = simulation(ctrl_cmd[1], x_hat, s, self.tracker2)
        x3, P3, s3 = simulation(ctrl_cmd[2], x_hat, s, self.tracker3)
        x4, P4, s4 = simulation(ctrl_cmd[3], x_hat, s, self.tracker4)
        x5, P5, s5 = simulation(ctrl_cmd[4], x_hat, s, self.tracker5)
        
        child = pybnb.Node()
        cost1 = compute_cost(P1)
        
        self.tmp_bound = self._bound
        tmp1 = [ctrl_cmd[0]]
        tmp2 = [ctrl_cmd[1]]
        tmp3 = [ctrl_cmd[2]]
        tmp4 = [ctrl_cmd[3]]
        tmp5 = [ctrl_cmd[4]]
        choices1 = self.choices + tmp1
        choices2 = self.choices + tmp2
        choices3 = self.choices + tmp3
        choices4 = self.choices + tmp4
        choices5 = self.choices + tmp5

        if len(choices1) == 4 or len(choices2) == 4 or len(choices3) == 4:
            self.value = self.value - DELTA #trick#TODO
        father_value = self.value

        child1_value = father_value + cost1
        child.state = (x1, s1, P1, child1_value, self.tmp_bound, choices1)
        yield child

        cost2 = compute_cost(P2)
        child2_value = father_value + cost2
        child = pybnb.Node()
        child.state = (x2, s2, P2, child2_value, self.tmp_bound, choices2)
        yield child

        cost3 = compute_cost(P3)
        child3_value = father_value + cost3
        child = pybnb.Node()
        child.state = (x3, s3, P3, child3_value, self.tmp_bound, choices3)
        yield child
        
        cost4 = compute_cost(P4)
        child4_value = father_value + cost4
        child = pybnb.Node()
        child.state = (x4, s4, P4, child4_value, self.tmp_bound, choices4)
        yield child

        cost5 = compute_cost(P5)
        child5_value = father_value + cost5
        child = pybnb.Node()
        child.state = (x5, s5, P5, child5_value, self.tmp_bound, choices5)
        yield child
        
        print('depth:',child.tree_depth)


    #
    # optional methods
    #
    def notify_solve_begins(self, comm, worker_comm, convergence_checker):
        pass

    def notify_new_best_node(self, node, current):
        
        pass

    def notify_solve_finished(self, comm, worker_comm, results):
        
        pass


def compute_cost(P):
   
    cost = np.trace(P)
    return cost

# init param (should be received from simulation)
x_hat = [1,1,1,1]
s = [5, 5, 0]
P = np.matrix([[1,0,0,0],
                              [0,1,0,0],
                              [0,0,1,0],
                              [0,0,0,1]])
T = 1
tracker1 = tracker.Tracker('1', True, P)
tracker2 = tracker.Tracker('2', True, P)
tracker3 = tracker.Tracker('3', True, P)
tracker4 = tracker.Tracker('3', True, P)
tracker5 = tracker.Tracker('3', True, P)


problem = Simple(x_hat, s, P, DELTA, tracker1, tracker2, tracker3, tracker4, tracker5)
solver = pybnb.Solver()
limit = len(ctrl_cmd)**4 + len(ctrl_cmd)**3 + len(ctrl_cmd)**2 + len(ctrl_cmd)**1 + 1

results = solver.solve(problem, node_limit=limit) 
best_node_states = results.best_node.state
print(results.best_node)
print(best_node_states[5])

#print(results.nodes)
# node limit =94
#absolute_gap=1e-9 #accettable gap between optimal objective and the found one.
