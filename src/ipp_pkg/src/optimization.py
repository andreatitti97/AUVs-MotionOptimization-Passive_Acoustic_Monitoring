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
# OPTIMIZATION PARAMETERS
key1, key2, key3, key4, key5 = -pi/12, -pi/15, 0, +pi/15, +pi/12 #-15°, -12°, 0°, +12°, +15°
DELTA = 10**15
ctrl_cmd = [key1, key2, key3, key4, key5]

class Target():
    def __init__(self, init_vector):
        self.x = init_vector[0]
        self.y = init_vector[1]
        
        self.vlx = init_vector[2]
        self.vly = init_vector[3]
        
        self.dt = config.OPTIMIZATION_TIME_STEP/8

    def update_state(self):

        self.x = self.x + self.vlx*self.dt
        self.y = self.y + self.vly*self.dt
        return [self.x, self.y, self.vlx, self.vly]

def simulation(control_input, target_est, platform_pose, phi, y, sensor, controller):

    tracker_ = tracker.Tracker('1', True, phi, y)
    #print('platform pose',platform_pose)
    #print('target_est',target_est)
    #print('control_input',control_input)
    #controller_auv = controller.Controller(1, config.CONTROLLER_GAIN) #0.01
    #platform = Platform(platform_pose, controller_auv)
    positions = np.zeros((4,2))
    tmp2 = np.array([platform_pose[0],platform_pose[0]])
    for i in range(0, 4):
        positions[i] = tmp2 + config.formation[i+1]
    orientations = np.zeros(4)
    for i in range(4):
        orientations[i] = platform_pose[2]
    target = Target(target_est)
    variable = 0
    meas_table = []
    j = 0
    propagation = False
    controller.update_leader_ori(platform_pose[2])
    for t in range(0,8):
        if t == 0:

            [platform_pose, tmp, positions, orientations] = controller.move_agents(platform_pose, control_input, config.OPTIMIZATION_TIME_STEP/8, positions, orientations,t,True)
            platform_pose = [platform_pose[0],platform_pose[1],tmp]
        else:
            [platform_pose, tmp, positions, orientations] = controller.move_agents(platform_pose, 0, config.OPTIMIZATION_TIME_STEP/8, positions, orientations,t,True)
            platform_pose = [platform_pose[0],platform_pose[1],tmp]
        target_state = target.update_state()
        if t >= 0 and t < 8:
            [measure_, rel_bearing_, meas_pos] = sensor[j].measureBearing(target_state[0],target_state[1],positions[j], orientations[j])
            arr = [variable,measure_,meas_pos[0],meas_pos[1]]
            meas_table.append(arr)
            j = j + 1
            if j == 4:
                propagation = True
                j = 0

        # update  WITH NEW MEASURAMENT
        if propagation ==  True:
            tracker_.processMeasurement(meas_table)
            tracker_.propagate_estimation(variable)
            propagation = False

        variable += config.OPTIMIZATION_TIME_STEP/8
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
    def __init__(self,x_hat, s, phi, y,initial_cost,sensors, cpf_control):
        # aggiungi un livello per imporre un orizzonte finito 
        self._x_hat = x_hat
        self._s = s
        tmp1 = phi.tolist()
        self._phi = []
        for i in range(len(y)):
            a = tmp1[0+(i*4):4+(i*4)]
            self._phi.append(a)
        self.__y = y.tolist()
        self.value = initial_cost #fake obj
        self._bound = 0 #lower bound 
        self._objective = 0 #real obj
        self.choices = []
        self.sensors = sensors
        self.controller = cpf_control

    # required methods
    def sense(self):
        return pybnb.minimize

    def objective(self):#TODO: L'OBJECTIVE E VALUE DEL NODO CHE È IL COSTO ACCUMULATO + IL NUOVO COSTO (vedi esempio knapsnack)
        return self.value

    def bound(self): # il bound è esclusivamente sull objective - CORRISPONDE AL COSTO ACCUMULATO FINO AL NODO IN ESAME
        #TODO il bound è dato dal solo costo accumulato, devi quindi calcolarlare il nuovo costo e fare  eventuali check 
        return self._bound

    def save_state(self, node):
        node.state = (self._x_hat, self._s, self._phi, self.__y, self.value, self._bound, self.choices)

    def load_state(self, node):
        (self._x_hat, self._s, self._phi, self.__y, self.value, self._bound, self.choices) = node.state

    def branch(self): #durante il branch devi calcolare le varie realizzazioni quindi simuli qua

        x_hat, s, phi, y = self._x_hat, self._s, self._phi, self.__y
        
        x1, phi1, y1, s1 = simulation(ctrl_cmd[0], x_hat, s, phi, y, self.sensors, self.controller)
        x2, phi2, y2, s2 = simulation(ctrl_cmd[1], x_hat, s, phi, y, self.sensors, self.controller)
        x3, phi3, y3, s3 = simulation(ctrl_cmd[2], x_hat, s, phi, y, self.sensors, self.controller)
        x4, phi4, y4, s4 = simulation(ctrl_cmd[3], x_hat, s, phi, y, self.sensors, self.controller)
        x5, phi5, y5, s5 = simulation(ctrl_cmd[4], x_hat, s, phi, y, self.sensors, self.controller)
        
        child = pybnb.Node()
        cost1 = compute_cost(phi1,len(y1))
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
        child.state = (x1, s1, phi1, y1, child1_value, self.tmp_bound, choices1)
        yield child

        cost2 = compute_cost(phi2,len(y2))
        child2_value = father_value + cost2
        child = pybnb.Node()
        child.state = (x2, s2, phi2, y2, child2_value, self.tmp_bound, choices2)
        yield child

        cost3 = compute_cost(phi3,len(y3))
        child3_value = father_value + cost3
        child = pybnb.Node()
        child.state = (x3, s3, phi3, y3, child3_value, self.tmp_bound, choices3)
        yield child
        
        cost4 = compute_cost(phi4,len(y1))
        child4_value = father_value + cost4
        child = pybnb.Node()
        child.state = (x4, s4, phi4, y4, child4_value, self.tmp_bound, choices4)
        yield child

        cost5 = compute_cost(phi5,len(y1))
        child5_value = father_value + cost5
        child = pybnb.Node()
        child.state = (x5, s5, phi5, y5, child5_value, self.tmp_bound, choices5)
        yield child
        if len(choices1) == 1:
            t_est_x.append(x1[0])
            t_est_y.append(x1[1])


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

    # Cooperative Path Following initialization
    cpf_control = cpf.CooperativePathFollowing(config.formation, config.N_AUV, config.PLATFORM_INIT_POSE[2], config.K_att, config.K_rep, config.d_rep, config.AUV_VEL)


    for i in range(config.N_AUV): 
        sensors.append(sensor.Sensor(str(i),1,0,0.005))#config.SIGMA_MEAS
    if config.OPTIMIZATION_ON == True:
        rospy.loginfo('STARTED OPTIMIZATION')
    
    while not rospy.is_shutdown():

        # INIT TARGET MODEL AND PLATFORM MODEL WITH THE LATEST ESTIMATION AND SENSOR POSITIONS 
        t_est = rospy.wait_for_message('/estimation',numpy_msg(Floats))
        s_state = rospy.wait_for_message('/platform_state',numpy_msg(Floats))
        regressor = rospy.wait_for_message('/regressor', numpy_msg(Floats))
        output = rospy.wait_for_message('/output', numpy_msg(Floats))
        t_est = t_est.data
        s_state = s_state.data
        regressor = regressor.data
        output = output.data

        # Compute the best solution solving the optimization with BnB or Greedy search
        problem = Simple(t_est, s_state,regressor,output, DELTA, sensors, cpf_control)
        solver = pybnb.Solver()
        limit = len(ctrl_cmd)**4 + len(ctrl_cmd)**3 + len(ctrl_cmd)**2 + len(ctrl_cmd)**1 + 1

        results = solver.solve(problem, node_limit=limit) 
        best_node_states = results.best_node.state
        ctrl_opt = best_node_states[6]
    
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
