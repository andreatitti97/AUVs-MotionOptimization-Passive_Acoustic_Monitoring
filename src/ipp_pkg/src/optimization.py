#Import basic system modules
import os
import pybnb
import importlib.util
import time
# Import math modules
import numpy as np
from math import cos, pi, sin, atan2
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
spec = importlib.util.spec_from_file_location("module.planner", class_path+"/spline_planner.py")
planner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(planner)
# FOLDER PATH DEFINITION
plot_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')
# Init global variables for callbacks
platform_state, target_est = [], []
t_est_x, t_est_y, auv = [], [], []
s_state_x, s_state_y = [], []

# Global Variable
DELTA = 10**15
cubicSpline = planner
DT = config.OPTIMIZATION_TIME_STEP#+config.MEAS_UPDATE*config.N_AUV#TODO:check
desired_vel = config.AUV_VEL

import matplotlib.pyplot as plt

def update_path(ax, ay, waypoint, init_theta):
        init_pose = [ax[-1],ay[-1]]

        theta_goal = init_theta+waypoint
        tmp_x = np.cos(theta_goal)*desired_vel*DT+init_pose[0]
        tmp_y = np.sin(theta_goal)*desired_vel*DT+init_pose[1]
        ax.append(tmp_x)
        ay.append(tmp_y)
        path = cubicSpline.CubicSpline2D(ax, ay)

        return path, ax, ay, theta_goal


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
    def __init__(self,x_hat, s,sensors, cpf_control, ctrl_cmds, ax, ay, distance_leader,cov):
        
        inf = float("inf")
        self.value = DELTA  
        self.initial_cost = DELTA
        self._bound = -inf # initial_cost-100 #lower bound 
        self.choices = []
        self._x_hat = x_hat
        self._s = s
        self.sensors = sensors
        self.controller = cpf_control
        self.ctrl_cmds = ctrl_cmds
        self.covariance = cov
        # Waypoints performed until optimization
        self.ax = ax 
        self.ay = ay
        self.distance_leader = distance_leader

    # required methods
    def sense(self):
        return pybnb.minimize

    def objective(self):#TODO: L'OBJECTIVE E VALUE DEL NODO CHE È IL COSTO ACCUMULATO + IL NUOVO COSTO (vedi esempio knapsnack)
        return self.value

    def bound(self): # il bound è esclusivamente sull objective - CORRISPONDE AL COSTO ACCUMULATO FINO AL NODO IN ESAME
        #TODO il bound è dato dal solo costo accumulato, devi quindi calcolarlare il nuovo costo e fare  eventuali check 
        return self._bound

    def save_state(self, node):
        
        node.state = (self._x_hat, self._s, self.value, self._bound, self.choices, self.ax, self.ay,self.distance_leader)

    def load_state(self, node):

        (self._x_hat, self._s, self.value, self._bound, self.choices, self.ax, self.ay,self.distance_leader) = node.state

    def branch(self): #durante il branch devi calcolare le varie realizzazioni quindi simuli qua

        x_hat, s, cov, ax, ay, d = self._x_hat, self._s, self.covariance, self.ax, self.ay, self.distance_leader
        diocane_x = []
        diocane_y = []
        for i in range(len(ax)):
            diocane_x.append(ax[i])
            diocane_y.append(ay[i])

        for i in range(config.U):

            x, phi, y, s, tmp_ax, tmp_ay, leader_distance = simulation(self.ctrl_cmds[i], x_hat, s, self.sensors, self.controller, ax, ay, d, cov)

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
            zoppo_x = []
            zoppo_y = []
            for i in range(len(diocane_x)):

                zoppo_x.append(diocane_x[i])
                zoppo_y.append(diocane_y[i])

            zoppo_x.append(tmp_ax)
            zoppo_y.append(tmp_ay)

            child.state = (x, s, child_value, self._bound, choices, zoppo_x, zoppo_y, leader_distance)
            ax.pop(-1)
            ay.pop(-1)
            
            yield child

            # Save data for debugging
            if len(choices) == 1:
      
                t_est_x.append(x[0])
                t_est_y.append(x[1])
                s_state_x.append(s[0])


def simulation(control_input, target_est, leader_pos, sensor, controller, ax, ay, leader_distance, covariance=[]):
    # Temporal Variable
    t, j = 0, 0 #time and counter init
    meas_table = []
    dt = config.OPTIMIZATION_TIME_STEP/config.time_scaler
    # Init classes for tracker and target
    target = Target(target_est, covariance=[])
    estimator = Estimation()
    # Load Path
    path = cubicSpline.CubicSpline2D(ax, ay)#re-generate the path followed up to now
    [rx, ry, ryaw, rk, s]=config.calc_spline_course(path,dt)
    path_index = len(ryaw)
    # Compute leader pose
    x,y = path.calc_position(leader_distance)
    yaw = path.calc_yaw(leader_distance)
    leader_pos =[x,y,yaw]
    # Compute agents pose
    positions = np.zeros((config.N_AUV,2))
    orientations = np.zeros(config.N_AUV)
    for i in range(config.N_AUV):

        if config.geometry == 'line' or config.geometry == 'line2':
            orientations[i] = leader_pos[2]
            positions[i,0] = leader_pos[0] + config.a*(config.formation[i,0]*np.cos(leader_pos[2])+config.formation[i,1]*np.sin(leader_pos[2]))
            positions[i,1] = leader_pos[1] + config.b*(-config.formation[i,0]*np.sin(leader_pos[2])+config.formation[i,1]*np.cos(leader_pos[2]))      
        if config.geometry == 'column' or config.geometry == 'column2':      
            distance_to_start = -config.formation[i]+leader_distance
            x,y = path.calc_position(distance_to_start)

            positions[i,0] = x
            positions[i,1] = y
            orientations[i] = path.calc_yaw(distance_to_start)

    for i in range(0,config.time_scaler):

        if i == 0:
            path, ax, ay, current_theta = update_path(ax,ay,control_input,leader_pos[2])

        [rx, ry, ryaw, rk, s] = config.calc_spline_course(path,dt)
        # Update  AUVs and target state
        leader_distance += desired_vel*dt
        [leader_pos, positions, orientations] = controller.move_agents(path, leader_distance, leader_pos, dt, positions, orientations, ryaw[path_index+i], True)

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
        t += dt
    [rx, ry, ryaw, rk, s] = config.calc_spline_course(path,dt)

    '''plt.subplots(1)
    plt.plot(ax, ay, "xb", label="Data points")
    plt.plot(leader_pos[0],leader_pos[1],'og',label='leader position')
    for i in range(config.N_AUV):
        plt.plot(positions[i,0],positions[i,1],'ok',label="AUV"+str(i))
    plt.plot(rx, ry, "-r", label="Cubic spline path")
    plt.legend()
    plt.axis('equal')
    plt.show() # uncomment for debugging'''

    return target.x, estimator.phi, estimator.y, leader_pos, ax[-1], ay[-1], leader_distance

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
    #cpf_control = cpf.CooperativePathFollowing(config.formation, config.N_AUV, config.PLATFORM_INIT_POSE[2], config.K_att, config.K_rep, config.d_rep, config.AUV_VEL)

    for i in range(config.N_AUV): 
        sensors.append(sensor.Sensor(str(i),1,0,0.000))#config.SIGMA_MEAS
    if config.OPTIMIZATION_ON == True:
        rospy.loginfo('STARTED OPTIMIZATION')
    
    while not rospy.is_shutdown():

        # INIT TARGET MODEL AND PLATFORM MODEL WITH THE LATEST ESTIMATION AND SENSOR POSITIONS 
        
        t_est = rospy.wait_for_message('/estimation',numpy_msg(Floats))
        s_state = rospy.wait_for_message('/platform_state',numpy_msg(Floats))
        ax = rospy.wait_for_message('/ax',numpy_msg(Floats))
        ay = rospy.wait_for_message('/ay',numpy_msg(Floats))
        cov = rospy.wait_for_message('/cov',numpy_msg(Floats))
        t_est = t_est.data
        s_state = s_state.data
        ax_array = ax.data
        ay_array = ay.data
        cov = cov.data
        
        ax, ay = [], []
        #ax_array.pop(-1) #remove the distance performed by leadeer (for now not seems not-useful)
        if config.geometry == 'column' or config.geometry == 'column2': # THIS CAN BECAME A FUNCTION
            if len(ay_array) <= 5:
                n = 5
            else:   
                n = len(ay_array)
                if n > 10:
                    n = 10

            for i in range(n):
                lenght = len(ay_array)-n
                ax.append(ax_array[lenght+i])
                ay.append(ay_array[lenght+i])
            
            path = cubicSpline.CubicSpline2D(ax, ay)
            
        elif config.geometry == 'line' or config.geometry == 'line2':
            for i in range(2):
                lenght = len(ay_array)-2
                ax.append(ax_array[lenght+i])
                ay.append(ay_array[lenght+i])
            path = cubicSpline.CubicSpline2D(ax, ay)
        distance_leader = path.s[-1]-1

        cpf_control = cpf.CooperativePathFollowing(config.formation, config.N_AUV, config.PLATFORM_INIT_POSE[2], config.K_att, config.K_rep, config.d_rep, config.AUV_VEL, True)
        n = len(t_est)
        covariance = np.zeros((n,n))
        for i in range(n):
            covariance[i,:] = cov[(i*n):(i*n)+n]

        ######## Compute the best solution solving the optimization with BnB or Greedy search #####
        problem = Simple(t_est, s_state, sensors, cpf_control, ctrl_cmd, ax,ay,distance_leader,covariance)
        solver = pybnb.Solver()
        ''' TEST ON BnB problem_simplified = Simple(t_est, s_state, DELTA, sensors, cpf_control, ctrl_cmd, covariance)
        results_preview = solver.solve(problem,queue_strategy="objective",node_limit=limit)
        lower_bound = results_preview.objective'''
        results = solver.solve(problem,queue_strategy="bound" ,node_limit=limit)#tnode_limit=limi #Uniform cost search con "objective"
        best_node_states = results.best_node.state #objective_stop=90000,time_limit=5
        wall_time = results.wall_time
        nodes = results.nodes
        avg_nodes.append(nodes)
        avg_time.append(wall_time)
        ctrl_opt = best_node_states[4]

        ###########################################################################################
        time.sleep(config.TIME_STEP*10)
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
            elif config.U == 7:
                ctrl_cmd = [-k_max, -k_max*4/(config.U),-k_max*2/(config.U),0,k_max*2/(config.U),k_max*4/(config.U),k_max]
            else:
                ctrl_cmd = [-k_max,0,k_max]
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