#Import basic system modules
import os
import pybnb
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
spec = importlib.util.spec_from_file_location("module.main2", "/home/andrea/ros_simulation_ws/src/ipp_pkg/src/main.py")
main = importlib.util.module_from_spec(spec)
spec.loader.exec_module(main)
TIME_SCALER = main.TIME_SCALER
TIME_STEP = main.TIME_STEP
MEAS_VARIANCE = main.MEAS_VARIANCE
TARGET_INIT = main.TARGET_INIT
BASELINE_X = main.BASELINE_X
BASELINE_Y = main.BASELINE_Y
tc = main.OPTIMIZATION_TIME_STEP
N_AUV = main.N_AUV
# FOLDER PATH DEFINITION
plot_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')
# Init global variables for callbacks
platform_state, target_est = [], []
t_est_x, t_est_y, auv = [], [], []
# OPTIMIZATION PARAMETERS
key1, key2, key3, key4, key5 = -pi/6, -pi/10, 0, +pi/10, +pi/6 #before 15 and 18
DELTA = 10**15
ctrl_cmd = [key1, key2, key3, key4, key5]

def sensorPlacement():
    for i in range(N_AUV): #TODO: AUV up to 6 consider
            if (i+1) % 2 == 0:
                if i+1 > 3:                    
                    auv.append(sensor.Sensor(str(i),1,0,MEAS_VARIANCE,
                        -1,BASELINE_X, BASELINE_Y-BASELINE_Y/2))#freq,mean,variance,displachement
                else:                 
                    auv.append(sensor.Sensor(str(i),1,0,MEAS_VARIANCE,
                        -1,0,BASELINE_Y))#freq,mean,variance,displachement
            if (i+1) % 2 == 1:
                if i+1 > 2:
                    auv.append(sensor.Sensor(str(i),1,0,MEAS_VARIANCE,
                        1,BASELINE_X, BASELINE_Y-BASELINE_Y/2))#freq,mean,variance,displachement
                else:   
                    auv.append(sensor.Sensor(str(i),1,0,MEAS_VARIANCE,
                        1,0,BASELINE_Y))#freq,mean,variance,displachement

class Platform():
    def __init__(self, init_vector):
        
        self.x = init_vector[0]
        self.y = init_vector[1]
        self.theta = init_vector[2]
        self.vl = 1
        self.dt = tc/2
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
        self.dt = tc/2

    def update_state(self):

        self.x = self.x + self.vlx*self.dt
        self.y = self.y + self.vly*self.dt
        return [self.x, self.y, self.vlx, self.vly]

def simulation(control_input, target_est, platform_pose, P):

    tracker_ = tracker.Tracker('1', True, N_AUV, P)
    platform = Platform(platform_pose)
    target = Target(target_est)
    
    for t in range(0,2):
        vehicle_pose = []
        rel_bearing = []
        measures = []
        if t == 0:
            platform_state = platform.update_state(control_input) 
        else:
            platform_state = platform.update_state(0) 
        target_state = target.update_state()

        for i in range(len(auv)):

            auv[i].vehiclePose(platform_state[0], platform_state[1], platform_state[2])
            auv[i].targetPoseReal(target_state[0], target_state[1], 0)
            [measure1, vehicle_pose1, rel_bearing1] = auv[i].measureBearing()
            measures.append(measure1)
            vehicle_pose.append(vehicle_pose1)
            rel_bearing.append(rel_bearing1)
    
        # update EKF WITH NEW MEASURAMENT
        tracker_.processMeasurement(measures,target_state, vehicle_pose, tc/2, True)
    [state, P] = tracker_.state
    state = [state[0,0], state[1,0], state[2,0], state[3,0]]
    return state, P, platform_state

def compute_cost(P):
   
    cost = np.trace(P)
    return cost

class Simple(pybnb.Problem):
    def __init__(self,x_hat, s, P, initial_cost):
        # aggiungi un livello per imporre un orizzonte finito 
        self._x_hat = x_hat

        self._s = s
        self._P = P
        self.value = initial_cost #fake obj
        self._bound = 0 #lower bound 
        self._objective = 0 #real obj
        self.choices = []

    # required methods
    def sense(self):
        return pybnb.minimize

    def objective(self):#TODO: L'OBJECTIVE E VALUE DEL NODO CHE È IL COSTO ACCUMULATO + IL NUOVO COSTO (vedi esempio knapsnack)
        return self.value

    def bound(self): # il bound è esclusivamente sull objective - CORRISPONDE AL COSTO ACCUMULATO FINO AL NODO IN ESAME
        #TODO il bound è dato dal solo costo accumulato, devi quindi calcolarlare il nuovo costo e fare  eventuali check 
        return self._bound

    def save_state(self, node):
        node.state = (self._x_hat, self._s, self._P, self.value, self._bound, self.choices)

    def load_state(self, node):
        (self._x_hat, self._s, self._P, self.value, self._bound, self.choices) = node.state

    def branch(self): #durante il branch devi calcolare le varie realizzazioni quindi simuli qua

        x_hat, s, P = self._x_hat, self._s, self._P
        
        x1, P1, s1 = simulation(ctrl_cmd[0], x_hat, s, P)
        x2, P2, s2 = simulation(ctrl_cmd[1], x_hat, s, P)
        x3, P3, s3 = simulation(ctrl_cmd[2], x_hat, s, P)
        x4, P4, s4 = simulation(ctrl_cmd[3], x_hat, s, P)
        x5, P5, s5 = simulation(ctrl_cmd[4], x_hat, s, P)

        child = pybnb.Node()
        cost1 = compute_cost(P1)
        
        self.tmp_bound = self._bound
        tmp1 = [ctrl_cmd[0]]
        choices1 = self.choices + tmp1
        tmp2 = [ctrl_cmd[1]]
        choices2 = self.choices + tmp2
        tmp3 = [ctrl_cmd[2]]
        choices3 = self.choices + tmp3
        tmp4 = [ctrl_cmd[3]]
        choices4 = self.choices + tmp4
        tmp5 = [ctrl_cmd[4]]
        choices5 = self.choices + tmp5

        if len(choices1) == 4 or len(choices2) == 4 or len(choices3) == 4:
            self.value = self.value - DELTA #trick#TODO
            #self.value = 0 #UNCOMMENT IF YO WANT THE LAST BEST NODE WITHOUT CONSIDERING COST 
            # AGGIUNGI CHE CONDIZIONE PER NODO CON COVARIANZA FINALE SINGOLA, NON DELLA SEQUENZA
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

        t_est_x.append(x1[0])
        t_est_y.append(x1[1])

def compute_cost(P):
    return np.trace(P)

def main():

    global covariance, target_est, platform_state
    # Ros Initialization
    rospy.init_node('optimization')
    pub = rospy.Publisher("ctrl_cmd",numpy_msg(Floats),queue_size=100)
    Hz = 1/(TIME_STEP)
    rate = rospy.Rate(Hz)
    # Init array and cov matrix
    ctrl_opt = []
    ctrl_plot = []
    P = np.eye((4))
    sensorPlacement() #recreate the AUV displachment
    rospy.loginfo('STARTED OPTIMIZATION')
    
    while not rospy.is_shutdown():

        # INIT TARGET MODEL AND PLATFORM MODEL WITH THE LATEST ESTIMATION AND SENSOR POSITIONS 
        t_est = rospy.wait_for_message('/estimation',numpy_msg(Floats))
        s_state = rospy.wait_for_message('/platform_state',numpy_msg(Floats))
        covariance_values = rospy.wait_for_message('/covariance_values', numpy_msg(Floats))
        t_est = t_est.data

        
        s_state = s_state.data
        covariance_values = covariance_values.data

        P = np.matrix([[10, 0, 0, 0], #TODO INITIAL COV VALUE AFTER LAST ESTIMATE - TO CHECK
                        [0, 10, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        # Compute the best solution solving the optimization with BnB or Greedy search
        problem = Simple(t_est, s_state, P, DELTA)
        solver = pybnb.Solver()
        limit = len(ctrl_cmd)**4 + len(ctrl_cmd)**3 + len(ctrl_cmd)**2 + len(ctrl_cmd)**1 + 1

        results = solver.solve(problem, node_limit=limit) 
        best_node_states = results.best_node.state

        # PRINT and PUBLISH results of optimization
        print(results.best_node)
        print('NODE CHOICHES:',best_node_states[5])
        ctrl_opt = best_node_states[5]
        pub.publish(np.array(ctrl_opt,dtype=np.float32))
        for i in range(4):
            ctrl_plot.append(ctrl_opt[i])
        #SAVE DATA FOR PLOT    
        np.savetxt(plot_path+'/plot_cmds.txt',ctrl_plot)
        np.savetxt(plot_path+'/t_est_x_opt.txt',t_est_x)
        np.savetxt(plot_path+'/t_est_y_opt.txt',t_est_y)
        ctrl_opt = []
        rate.sleep()
    
if __name__ == '__main__':
    
    main()
    
