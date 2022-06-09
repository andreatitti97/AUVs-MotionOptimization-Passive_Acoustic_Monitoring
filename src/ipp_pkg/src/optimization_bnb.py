#Import basic system modules
import os
import pybnb
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

#GLOBAL VARIABLES - simulation parameters
key1 = -pi/12
key2 = 0
key3 = pi/12
key4 = pi/6
key5 = -pi/6
keys = [key1, key2, key3, key4, key5]
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
target_theta = pi/4 #TARGET_INIT[2]
velTarget = 3 #TARGET_INIT[3]
key1 = -pi/12
key2 = 0
key3 = pi/12

ctrl_cmd = [key1, key2, key3]
tc = 8
sensor1 = sensor.Sensor('first_streamer',1,0,0,1)
sensor2 = sensor.Sensor('seconda_streamer',1,0,0,-1)
count = []
class Platform():
    def __init__(self, init_vector):
        self.x = init_vector[0]
        self.y = init_vector[1]
        self.theta = init_vector[2]
        self.vl = 1
        self.dt = tc
    def update_state(self, delta):

        self.dt = tc
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

        self.dt = tc
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
    
    [measure1,sensor_pose1, rel_bearing1] = sensor1.measureBearing()
    [measure2,sensor_pose2, rel_bearing2] = sensor2.measureBearing()
    measures = [measure1, measure2]
    # update EKF
    
    tracker.processMeasurement(measures,target_state, sensor_pose1, sensor_pose2, tc)
    [state, P] = tracker.state
    #print('traccia matrice:',np.trace(P))
    #print('stato associato;',state)
    state = [state[0,0], state[1,0], state[2,0], state[3,0]]
    return state, P, platform_state, target_state

def compute_cost(P):
   
    cost = np.trace(P)
    return cost

class Simple(pybnb.Problem):
    def __init__(self, x_hat, s, P, initial_cost, tracker1, tracker2, tracker3):
        # aggiungi un livello per imporre un orizzonte finito 
        self._x_hat = x_hat
        self._s = s
        self._P = P
        self.weight = 0
        self.value = initial_cost
        self.level = 0
        self._bound = 0 #lower bound 
        self.choices = []
        self.tracker1 = tracker1
        self.tracker2 = tracker2
        self.tracker3 = tracker3
        self.tmp = 0
    #
    # required methods
    #
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
        node.state = (self._x_hat, self._s, self._P, self.value, self._bound)

    def load_state(self, node):
        (self._x_hat, self._s, self._P, self.value, self._bound) = node.state

    def branch(self): #durante il branch devi calcolare le varie realizzazioni quindi simuli qua
        # qui carica lo stato del nodo padre e genera 3 figli a cui assegnare i vari costi e stati, ricorda che devi far ereditare
        # anche le realizzazioni del target e della piattaforma e P, non solo il costo.
        x_hat, s, P = self._x_hat, self._s, self._P
        ##print('FATHER ',x_hat,s)
        
        x1, P1, s1, x_real1 = simulation(ctrl_cmd[0], x_hat, s, self.tracker1)
        x2, P2, s2, x_real2 = simulation(ctrl_cmd[1], x_hat, s, self.tracker2)#TODO tracker
        x3, P3, s3, x_real3 = simulation(ctrl_cmd[2], x_hat, s, self.tracker3)
        #print(x_real1, x_real2, x_real3)
        time.sleep(1)
        father_value = self.value
        child = pybnb.Node()
        cost1 = compute_cost(P1)
        self.tmp += 0
        child1_value = father_value + cost1
        print(child1_value,father_value)
        child.state = (x_real1, s1, P1, child1_value, self.tmp)
        yield child
        cost2 = compute_cost(P2)
        child2_value = father_value + cost2
        child = pybnb.Node()
        child.state = (x_real2, s2, P2, child2_value, self.tmp)
        yield child
        cost3 = compute_cost(P3)
        child3_value = father_value + cost3
        child = pybnb.Node()
        child.state = (x_real3, s3, P3, child3_value, self.tmp)
        yield child
        # PRINT FOR DEBUGGING
        #print(P1,P2,P3)
        print('state:',child.state)
        print('depth:',child.tree_depth)
        #tmp = child.tree_depth
        #print(cost1,cost2,cost3)
        #print(x1,x2,x3)
        #print(s1,s2,s3)
        

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
        

        rospy.sleep(TIME_STEP*10)

        problem = Simple(t_est, s_state, P, np.trace(P), tracker1, tracker2, tracker3)
        solver = pybnb.Solver()
        results = solver.solve(problem, node_limit=50) #accettable gap between optimal objective and the found one.
        print(results.best_node)
        pub.publish(np.array(ctrl_cmd,dtype=np.float32))

        #SAVE FILE FOR PLOT    
        np.savetxt(lib_path+'/ctrl_cmd.txt',ctrl_cmd)
        np.savetxt(plot_path+'/target_traj_est_x.txt',target_traj_est_x)
        np.savetxt(plot_path+'/target_traj_est_y.txt',target_traj_est_y)
        np.savetxt(plot_path+'/target_traj_real_x.txt',target_traj_real_x)
        np.savetxt(plot_path+'/target_traj_real_y.txt',target_traj_real_y)
        np.savetxt(plot_path+'/plot_cmds.txt',ctrl_plot)
        rospy.loginfo(ctrl_cmd)
        ctrl_cmd = []
        rate.sleep()
 
if __name__ == '__main__':
    
    main()
