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
spec = importlib.util.spec_from_file_location("module.tracker_optimization", class_path+"/tracker_cassino.py")
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)
spec = importlib.util.spec_from_file_location("module.sensor_cpf", class_path+"/sensor_cpf.py")
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

# Global Variables
DELTA = 10**15
M = config.M #planning horizon
opt_scaler = config.STATE_PROPAGATION

class Target():
    def __init__(self, init_vector):
        self.x = init_vector[0]
        self.y = init_vector[1]
        
        self.vlx = init_vector[2]
        self.vly = init_vector[3]
        self.dt = config.OPTIMIZATION_TIME_STEP/opt_scaler

    def update_state(self):

        self.x = self.x + self.vlx*self.dt
        self.y = self.y + self.vly*self.dt
        return [self.x, self.y, self.vlx, self.vly]

def simulation(control_input, target_est, platform_pose, sensor, controller):

    tracker_ = tracker.Tracker('opt', True)
    positions = np.zeros((4,2))
    tmp2 = np.array([platform_pose[0],platform_pose[1]])
    for i in range(0, 4):
        positions[i] = tmp2 + config.formation[i+1]
    orientations = np.zeros(4)
    for i in range(4):
        orientations[i] = platform_pose[2]
    target = Target(target_est)
    t = 0
    meas_table = []
    j = 0
    controller.update_leader_ori(platform_pose[2])
    for i in range(0,opt_scaler):
        if i == 0:
            cmd = control_input
        else:
            cmd = 0
        # Update AUVs and target state
        [platform_pose, tmp, positions, orientations] = controller.move_agents(platform_pose, cmd, config.OPTIMIZATION_TIME_STEP/opt_scaler, positions, orientations,i,True, opt_scaler)
        platform_pose = [platform_pose[0],platform_pose[1],tmp]
        target_state = target.update_state()
        if i >= 0 and i < (opt_scaler-1):
            [measure_, rel_bearing_, meas_pos] = sensor[j].measureBearing(target_state[0],target_state[1],positions[j], orientations[j])
            arr = [t,measure_,meas_pos[0],meas_pos[1]]
            meas_table.append(arr)
            j = j + 1
            if j == 4:
                #propagation = True
                j = 0

        # update  WITH NEW MEASURAMENT
        else: 
            tracker_.processMeasurement(meas_table)
            tracker_.propagate_estimation(t)

        t += config.OPTIMIZATION_TIME_STEP/opt_scaler
    [state, phi, y] = tracker_.state
    state = [state[0,0], state[1,0], state[2,0], state[3,0]]
    return state, phi, y, platform_pose #platform_state

def compute_cost(phi,length_y):

    tmp_phi = np.zeros((length_y,4))
    for i in range(length_y):
        a = phi[i]
        tmp_phi[i,:] = [a[0],a[1],a[2],a[3]]

    A = np.dot(np.transpose(tmp_phi),tmp_phi)
    cost = np.linalg.norm(np.linalg.inv(A))*np.linalg.norm(A)
    return cost

class Simple(pybnb.Problem):
    def __init__(self,x_hat, s, initial_cost,sensors, cpf_control, ctrl_cmds):
        # aggiungi un livello per imporre un orizzonte finito 
        self._x_hat = x_hat
        self._s = s
        
        self.value = initial_cost #fake obj
        self._bound = 0 #lower bound 
        self._objective = 0 #real obj
        self.choices = []
        self.sensors = sensors
        self.controller = cpf_control
        self.ctrl_cmds = ctrl_cmds

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

        x_hat, s = self._x_hat, self._s
        
        x1, phi1, y1, s1 = simulation(self.ctrl_cmds[0], x_hat, s, self.sensors, self.controller)
        x2, phi2, y2, s2 = simulation(self.ctrl_cmds[1], x_hat, s, self.sensors, self.controller)
        x3, phi3, y3, s3 = simulation(self.ctrl_cmds[2], x_hat, s, self.sensors, self.controller)
        x4, phi4, y4, s4 = simulation(self.ctrl_cmds[3], x_hat, s, self.sensors, self.controller)
        x5, phi5, y5, s5 = simulation(self.ctrl_cmds[4], x_hat, s, self.sensors, self.controller)
        
        child = pybnb.Node()
        cost1 = compute_cost(phi1,len(y1))
        self.tmp_bound = self._bound
        tmp1 = [self.ctrl_cmds[0]]
        choices1 = self.choices + tmp1
        tmp2 = [self.ctrl_cmds[1]]
        choices2 = self.choices + tmp2
        tmp3 = [self.ctrl_cmds[2]]
        choices3 = self.choices + tmp3
        tmp4 = [self.ctrl_cmds[3]]
        choices4 = self.choices + tmp4
        tmp5 = [self.ctrl_cmds[4]]
        choices5 = self.choices + tmp5

        if len(choices1) == M or len(choices2) == M or len(choices3) == M:
            self.value = self.value - DELTA #trick#TODO
            #self.value = 0 #UNCOMMENT IF YO WANT THE LAST BEST NODE WITHOUT CONSIDERING COST 
            # AGGIUNGI CHE CONDIZIONE PER NODO CON COVARIANZA FINALE SINGOLA, NON DELLA SEQUENZA
        father_value = self.value

        child1_value = father_value + cost1
        child.state = (x1, s1, child1_value, self.tmp_bound, choices1)
        yield child

        cost2 = compute_cost(phi2,len(y2))
        child2_value = father_value + cost2
        child = pybnb.Node()
        child.state = (x2, s2, child2_value, self.tmp_bound, choices2)
        yield child

        cost3 = compute_cost(phi3,len(y3))
        child3_value = father_value + cost3
        child = pybnb.Node()
        child.state = (x3, s3, child3_value, self.tmp_bound, choices3)
        yield child
        
        cost4 = compute_cost(phi4,len(y4))
        child4_value = father_value + cost4
        child = pybnb.Node()
        child.state = (x4, s4, child4_value, self.tmp_bound, choices4)
        yield child

        cost5 = compute_cost(phi5,len(y5))
        child5_value = father_value + cost5
        child = pybnb.Node()
        child.state = (x5, s5, child5_value, self.tmp_bound, choices5)
        yield child
        if len(choices1) == 1:
            t_est_x.append(x4[0])
            t_est_y.append(x4[1])
            s_state_x.append(s1[0])
            s_state_y.append(s1[1])


def main():

    global target_est
    # Ros Initialization
    rospy.init_node('optimization')

    pub = rospy.Publisher("ctrl_cmd",numpy_msg(Floats),queue_size=100)
    Hz = 1/(config.TIME_STEP)
    rate = rospy.Rate(Hz)
    # Init array and cov matrix
    ctrl_opt = []
    ctrl_plot = []
    #sensorPlacement()
    sensors = []
    old_ctrls = []
    # OPTIMIZATION PARAMETERS
    count_low = 0
    count_max = 0
    config.delta_k = 3*pi/180 
    config.U = 7
    ctrl_cmd = [-config.k_max , -config.k_max *4/(config.U),
                -config.k_max *2/(config.U),
                0,
                config.k_max *2/(config.U),
                config.k_max *4/(config.U),config.k_max ]
    limit = 0.0
    for i in range(M+1):
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
        t_est = t_est.data
        s_state = s_state.data

        ######## Compute the best solution solving the optimization with BnB or Greedy search #####
        problem = Simple(t_est, s_state, DELTA, sensors, cpf_control, ctrl_cmd)
        solver = pybnb.Solver()
        results = solver.solve(problem, node_limit=limit) 
        best_node_states = results.best_node.state

        ctrl_opt = best_node_states[4]
        ###########################################################################################

        pub.publish(np.array(ctrl_opt,dtype=np.float32))
        ctrl_plot.append(ctrl_opt[0])
        old_ctrls.append(ctrl_opt[0])
        # Adapt online the heading changes:
        if len(old_ctrls) == 3:
            for i in range(len(old_ctrls)):
                if [0-(1e-3)] <=  np.abs(old_ctrls[i])-(1e-3) <= config.k_max *2/(config.U):
                    count_low += 1 
                    print(count_low)
                    if count_low == 3:
                        config.k_max  = config.k_max  - config.delta_k
                        count_low = 0

                elif np.abs(old_ctrls[i]) >= config.k_max :
                    count_max += 1
                    if count_max == 3:
                        config.k_max  = config.k_max  + config.delta_k
                        count_max = 0
                        
            count_low = 0
            count_max = 0
            old_ctrls = []
            ctrl_cmd = [-config.k_max , -config.k_max *4/(config.U),
                        -config.k_max *2/(config.U),0,config.k_max *2/(config.U),
                        config.k_max *4/(config.U),config.k_max ]
        #SAVE DATA FOR PLOT    
        np.savetxt(plot_path+'/plot_cmds.txt',ctrl_plot)
        np.savetxt(plot_path+'/t_est_x_opt.txt',t_est_x)
        np.savetxt(plot_path+'/t_est_y_opt.txt',t_est_y)
        np.savetxt(plot_path+'/s_state_x.txt',s_state_x)
        np.savetxt(plot_path+'/s_state_y.txt',s_state_y)
        ctrl_opt = []
        rate.sleep()
    
if __name__ == '__main__':
    
    main()
