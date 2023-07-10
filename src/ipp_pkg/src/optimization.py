#Import basic system modules
import os
import pybnb
import importlib.util
import time
# Import math modules
import numpy as np
from math import cos, pi, sin
# Import ROS modules and Service
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

# Import costum classes
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
spec = importlib.util.spec_from_file_location("module.sensor_cpf", class_path+"/sensor.py")
sensor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor)
spec = importlib.util.spec_from_file_location("module.cpf", class_path+"/cpf.py")
cpf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cpf)
# FOLDER PATH DEFINITION
plot_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')
# Init global variables for callbacks
platform_state, target_est = [], []
t_est_x, t_est_y, auv = [], [], []
s_state_x, s_state_y = [], []

# Global Variable
DELTA = 10**15


class Target():
    def __init__(self, init_state, covariance=[]):
        'INPUT: estimate state of the target, eventualy associate covariance for uscented transform'
        self.x = init_state
        self.dt = config.OPTIMIZATION_TIME_STEP/config.time_scaler
        self.cov = covariance
        self.F = np.matrix([[1,0,self.dt,0], # Target State Transition Matrix - CV
                        [0,1,0,self.dt],
                        [0,0,1,0],
                        [0,0,0,1]])

class Estimation():
    def __init__(self):
        self.phi = []
        self.y = []
        self.x = np.zeros((2,1))
    
    def regressorUpdate(self, measure, auv_position_x, auv_position_y):

        self.y.append(auv_position_x*np.sin(measure) - auv_position_y*np.cos(measure))
        C = [np.sin(measure), -np.cos(measure)]
        self.phi.append(C)

    def computeState(self,table):
        for i in range(len(table)):
            input_meas = table[i]
            self.regressorUpdate(input_meas[0],input_meas[1],input_meas[2])
        # Re-arrange python list into numpy array for matrix moltiplication
        tmp_y = np.zeros((len(self.y),1))
        for i in range(len(self.y)):
            tmp_y[i] = self.y[i]
        # Compute State (OPTIONAL, in this case we need only the conditioning to assigne the cost)
        self.x = np.dot(np.linalg.pinv(self.phi),tmp_y)

class Simple(pybnb.Problem):
    def __init__(self,x_hat, s, initial_cost,sensors, cpf_control, ctrl_cmds, cov):
        
        inf = float("inf")
        self.value = initial_cost  
        self.initial_cost = initial_cost
        self._bound = -inf # initial_cost-100 #lower bound 
        self.choices = []
        self._x_hat = x_hat
        self._s = s
        self.sensors = sensors
        self.controller = cpf_control
        self.ctrl_cmds = ctrl_cmds
        self.covariance = cov

    # required methods
    def sense(self):
        return pybnb.minimize

    def objective(self):#TODO: L'OBJECTIVE E VALUE DEL NODO CHE È IL COSTO ACCUMULATO + IL NUOVO COSTO (vedi esempio knapsnack)
        return self.value

    def bound(self): # il bound è esclusivamente sull objective - CORRISPONDE AL COSTO ACCUMULATO FINO AL NODO IN ESAME
        #TODO il bound è dato dal solo costo accumulato, devi quindi calcolarlare il nuovo costo e fare  eventuali check 
        return self._bound

    def save_state(self, node):
        node.state = (self._x_hat, self._s, self.value, self._bound, self.choices)

    def load_state(self, node):
        (self._x_hat, self._s, self.value, self._bound, self.choices) = node.state

    def branch(self): #durante il branch devi calcolare le varie realizzazioni quindi simuli qua

        x_hat, s, cov = self._x_hat, self._s, self.covariance
        
        for i in range(config.U):

            x, phi, y, s = simulation(self.ctrl_cmds[i], x_hat, s, self.sensors, self.controller, cov)
            # Update the sequence of control decisions
            tmp = [self.ctrl_cmds[i]]
            choices = self.choices + tmp
            # If we reached the planning horizon we subtract the DELTA to stop the algorithm and choose only terminal nodes
            if len(choices) == config.M:
                self.value = self.value - self.initial_cost ##THIS IS MANDATORY FOR ADDITIVE COST ALONG THE SEQUENCE
                self._bound = self.value #- cost1 
            father_value = self.value #THIS IS MANDATORY FOR ADDITIVE COST ALONG THE SEQUENC

            # Add the cost of the node to the sequence
            cost = compute_cost(phi,len(y))
            child_value = father_value + cost
            # Branch the tree
            child = pybnb.Node()
            child.state = (x, s, child_value, self._bound, choices)
            yield child
            #time.sleep(2)
            # Save data for debugging
            if len(choices) == 1:
      
                t_est_x.append(x[0])
                t_est_y.append(x[1])
                s_state_x.append(s[0])

def simulation(control_input, target_est, leader_pos, sensor, controller, covariance=[]):

    # Init classes for tracker and target
    target = Target(target_est, covariance=[])
    estimator = Estimation()
    # Load agents state
    positions = np.zeros((config.N_AUV,2))
    for i in range(0,config.N_AUV):       
        positions[i,0] = leader_pos[0] + config.a*(config.formation[i+1,0]*np.cos(config.PLATFORM_INIT_POSE[2])+config.formation[i+1,1]*np.sin(config.PLATFORM_INIT_POSE[2]))
        positions[i,1] = leader_pos[1] + config.b*(-config.formation[i+1,0]*np.sin(config.PLATFORM_INIT_POSE[2])+config.formation[i+1,1]*np.cos(config.PLATFORM_INIT_POSE[2]))
    orientations = np.zeros(config.N_AUV)
    for i in range(config.N_AUV):
        orientations[i] = leader_pos[2]
    controller.update_leader_ori(leader_pos[2])
    # Temporal Variable
    t, j = 0, 0 #time and counter init
    meas_table = []
    for i in range(0,config.time_scaler):
        
        if i == 0:
            cmd = control_input
        else:
            cmd = 0
        # Update AUVs and target state
        [leader_pos, tmp, positions, orientations, des_pose] = controller.move_agents(leader_pos, cmd, config.OPTIMIZATION_TIME_STEP/config.time_scaler, positions, orientations,i,True, config.time_scaler)
        leader_pos = [leader_pos[0],leader_pos[1],tmp]

        tmp = np.zeros((4,1))
        for j in range(4):  
            tmp[j]=target.x[j]
        target.x = target.F*tmp

        if i == (config.time_scaler-2):
            for j in range(config.N_AUV):
                [measure_, rel_bearing_, meas_pos] = sensor[j].measureBearing(target.x[0],target.x[1],positions[j],orientations[j])
                arr = [measure_,meas_pos[0],meas_pos[1]]
                meas_table.append(arr)
        # update WITH NEW MEASURAMENT
        elif i == (config.time_scaler-1):     
            estimator.computeState(meas_table)
        t += config.OPTIMIZATION_TIME_STEP/config.time_scaler

    return target.x, estimator.phi, estimator.y, leader_pos

def compute_cost(phi,length_y):

    tmp_phi = np.zeros((length_y,2))
    for i in range(length_y):
        a = phi[i]
        tmp_phi[i,:] = [a[0],a[1]]
  
    E = np.dot(np.transpose(tmp_phi),tmp_phi)
    cost2 = np.linalg.norm(np.linalg.inv(E),2)*np.linalg.norm(E,ord=2)
    return cost2


def main():

    global target_est
    # Ros Initialization
    rospy.init_node('optimization')

    pub = rospy.Publisher("ctrl_cmd",numpy_msg(Floats),queue_size=100)
    Hz = 1/(config.TIME_STEP)
    rate = rospy.Rate(Hz)
    # Init array and cov matrix
    ctrl_opt, ctrl_plot, sensors, old_ctrls = [], [], [], []  
    avg_time, avg_nodes = [],[]
    # OPTIMIZATION PARAMETERS
    count_low, count_max = 0,0
    ctrl_cmd = config.ctrl_cmd
    limit = 0.0
    k_max = config.k_max
    delta_k = config.delta_k
    for i in range(config.M+1):
        limit += config.U**i
 
    # Cooperative Path Following initialization
    cpf_control = cpf.CooperativePathFollowing(config.formation, config.N_AUV, config.PLATFORM_INIT_POSE[2], config.K_att, config.K_rep, config.d_rep, config.AUV_VEL)

    for i in range(config.N_AUV): 
        sensors.append(sensor.Sensor(str(i),1,0,0.000))#config.SIGMA_MEAS
    if config.OPTIMIZATION_ON == True:
        rospy.loginfo('STARTED OPTIMIZATION')
    
    while not rospy.is_shutdown():

        # INIT TARGET MODEL AND PLATFORM MODEL WITH THE LATEST ESTIMATION AND SENSOR POSITIONS 
        
        t_est = rospy.wait_for_message('/estimation',numpy_msg(Floats))
        s_state = rospy.wait_for_message('/platform_state',numpy_msg(Floats))
        cov = rospy.wait_for_message('/covariance',numpy_msg(Floats))
        t_est = t_est.data
        s_state = s_state.data
        cov = cov.data
        n = len(t_est)
        covariance = np.zeros((n,n))

        for i in range(n):
            covariance[i,:] = cov[(i*n):(i*n)+n]

        ######## Compute the best solution solving the optimization with BnB or Greedy search #####
        problem = Simple(t_est, s_state, DELTA, sensors, cpf_control, ctrl_cmd, covariance)
        solver = pybnb.Solver()
        ''' TEST ON BnB problem_simplified = Simple(t_est, s_state, DELTA, sensors, cpf_control, ctrl_cmd, covariance)
        results_preview = solver.solve(problem,queue_strategy="objective",node_limit=limit)
        lower_bound = results_preview.objective'''
        results = solver.solve(problem,queue_strategy="objective" ,node_limit=limit)#tnode_limit=limi #Uniform cost search con "objective"
        best_node_states = results.best_node.state #objective_stop=90000,time_limit=5
        wall_time = results.wall_time
        nodes = results.nodes
        avg_nodes.append(nodes)
        avg_time.append(wall_time)
        ctrl_opt = best_node_states[4]

        ###########################################################################################

        pub.publish(np.array(ctrl_opt,dtype=np.float32))
        ctrl_plot.append(ctrl_opt[0])
        old_ctrls.append(ctrl_opt[0])
        # Adapt online the heading changes: # TO DEBUG !!!!! OR TO TUNE PROPERLY -  in theory done to check
        if len(old_ctrls) == 3:
            for i in range(len(old_ctrls)):
                if [0-(1e-3)] <=  np.abs(old_ctrls[i])-(1e-3) <= config.k_max *2/(config.U):
                    count_low += 1 

                    if count_low == 3:
                        if k_max <= 8*pi/180:
                            k_max = k_max
                            count_low = 0
                        else:
                            print('DECREASING K_MAX----------------------------')
                            k_max  = k_max  - delta_k
                            count_low = 0

                elif np.abs(old_ctrls[i]) >= k_max :
                    count_max += 1
                    if count_max == 3:
                        if k_max >= 25*pi/180:
                            k_max = k_max
                            count_max = 0
                        else:
                            print('INCREASING K_MAX++++++++++++++++++++++++++++')
                            k_max  = k_max  + delta_k
                            count_max = 0
                        
            count_low = 0
            count_max = 0
            old_ctrls = []
            if config.U == 5: 
                ctrl_cmd = [-k_max, -k_max*4/(config.U),0,k_max*4/(config.U),k_max]
            else:
                ctrl_cmd = [-k_max, -k_max*4/(config.U),-k_max*2/(config.U),0,k_max*2/(config.U),k_max*4/(config.U),k_max]

            print('COUNT LOW-------------------------------',count_low)
            print('COUNT MAX+++++++++++++++++++++++++++++++',count_max)

        #SAVE DATA FOR PLOT
        np.savetxt(plot_path+'/plot_cmds.txt',ctrl_plot)
        np.savetxt(plot_path+'/t_est_x_opt.txt',t_est_x[0])
        np.savetxt(plot_path+'/t_est_y_opt.txt',t_est_y[0])
        np.savetxt(plot_path+'/s_state_x.txt',s_state_x)
        np.savetxt(plot_path+'/s_state_y.txt',s_state_y)

        np.savetxt(plot_path+'/wall_times.txt',avg_time)
        np.savetxt(plot_path+'/nodes.txt',avg_nodes)
        ctrl_opt = []
        rate.sleep()
    
if __name__ == '__main__':
    
    main()