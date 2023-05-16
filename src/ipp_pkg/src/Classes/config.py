from math import pi
import numpy as np
# Simulation parameters
TIME_DURATION = 300 # (s)
TIME_STEP = 0.01
TIME_SCALER = 80 # MAX for communication purpose 
OPTIMIZATION_ON = False

# Estimation Parameters
TP = 40 # regressor MAX length
STATE_PROPAGATION = 6 #time between each propagation of the estimation (in real case a consensus (?))
SIGMA_MEAS = 0.002 # uncertainty = 5° --> sigma^2 = (uncertainty*pi/180)^2
MEAS_UPDATE = 1#3 #STATE_PROPAGATION/4 # MEAS_UPDATE (s)= meas_update*(TIME_SCALER*TIME_STEP)

# Team parameter: number of agents, baselines_XY, inital position, type of formation
N_AUV = 2
PLATFORM_INIT_POSE = [0, 0, 0] #[x,y,theta]
AUV_VEL = 2.0 #(m/s)
AUV_MAX_VEL = 4.0 #(m/s)
GAIN_YAW_RATE = 0.1 

# PARTE SEMPRE DA UNA DISTANZA COMPRESA TRA I 3.5 E 5 KM con velocità da 4 a 8 m/s
# Target parameter: start, goal, min max vels

#TARGET_INIT = [+1000,-2500, pi, 6.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 1
#TARGET_INIT = [3000,-1500,pi/2,9.0]#[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 2
#TARGET_INIT = [4000, 200, 140*pi/180, 8.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 3
#TARGET_INIT = [-3000, -2000, pi/2, 7.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 4
#TARGET_INIT = [-1500, 2000, pi/8, 5.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 5


TARGET_INIT = [400,-200, pi/2, 3.0]

TARGET_GOAL = [100, 100,TARGET_INIT[2]]
TARGET_VEL = TARGET_INIT[3] #(m/s)
MAX_TARGET_VEL = 3 #(m/s) (only if target no costant vels)
MIN_TARGET_VEL = 3 #(m/s)

# Communication Paramaters (to generalize) - for now based on v-sense data
s = 1 #0.5
mean = [3.5/s, 2.0/s, 1.0/s, 0.0]  #medium latencies between each AUV and the 4th (in fact latencies 0.0 for the 4th).
variance = [0.6, 0.4, 0.2, 0.0] #the same as before vor the variances.

# Optimization Parameters
OPTIMIZATION_TIME_STEP = STATE_PROPAGATION*(TIME_SCALER*TIME_STEP) #VA INTESO COME delta_k (planning stage)in secondi
k_max = 15*pi/180 
delta_k = 0#3*pi/180 
U = 5 #number of control choices
M = 3 # planning horizon
ctrl_cmd = [-k_max, -k_max*4/(U),0,k_max*4/(U),k_max] #set of control actions
#ctrl_cmd = [-k_max, -k_max*4/(U),-k_max*2/(U),0,k_max*2/(U),k_max*4/(U),k_max] #set of control actions

# Cooperative Path Following Params
a, b = 1, -1
K_att = 1.0# for in line -> 0.05 # Attractive gain
K_rep = 0.0 #1.0 #for in line -> 10.0 # Repulsive gain
d_rep = 150 # Distance threshold for repulsion
# CHOOSE THE GEOMETRY BETWEEN THE AGENTS
geometry = 'column2'

if geometry == 'line':
    formation =  np.array([[PLATFORM_INIT_POSE[0], PLATFORM_INIT_POSE[1]],
                            [0, 300],
                            [0,-100], 
                            [0, 100], 
                            [0,-300]]) # IN LINEA 
    
elif geometry == 'column':
    formation = np.array([[PLATFORM_INIT_POSE[0], PLATFORM_INIT_POSE[1]],
                            [-600, 0], 
                            [-400,0], 
                            [-200,0], 
                            [0,0]]) # IN COLONNA
    
elif geometry == 'polygon':
    formation =  np.array([[PLATFORM_INIT_POSE[0], PLATFORM_INIT_POSE[1]],
                            [170,-200],
                            [0, 100], 
                            [0,-100], 
                            [170,200]]) # TRAPEZOIDALE
    
elif geometry == 'line2':
    formation =  np.array([[PLATFORM_INIT_POSE[0], PLATFORM_INIT_POSE[1]],
                            [0, -25],
                            [0, +25]]) # TRAPEZOIDALE
    
elif geometry == 'column2':
    formation =  np.array([[PLATFORM_INIT_POSE[0], PLATFORM_INIT_POSE[1]],
                            [-5, 0],
                            [-55, 0]]) # TRAPEZOIDALE 


'REMARK ABOUT SIMULATION TIME & COMMUNICATION PERFORMANCES'
'''
The time is scaled, so the following values are to be considered for the int counter,
therefore real value in sec multiply for 0.8.

Communication Statistics: 8-40 bits/s End-To-End from 1km to 25 m (each node) / 4/100 bits/s Point-To-Point  from 1 km to 25 m
max nodes distance = 1 km 
we can assume a 64bits/s communication rate PTP (quite bad)
therefore a messagge of 4 double take 256bit then 4 seconds. 
from agent A to D we take 16 secondi then send back the Ctrl Cmd (one float 64bit)
more or less 4 seconds. 

TOTAL communication time (without considering navigation data, will be another type of messages)
is aout 16+4 = c.a. 20 seconds then we can trigger the optimization
while before propagate the estimation around 16 seconds
'''

class Pose:
    """2D pose"""

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

def generatePolynomialTrajectory(ts, y_from, yd_from, ydd_from, y_to, yd_to, ydd_to):
        
        a0 = y_from
        a1 = yd_from
        a2 = ydd_from / 2

        a3 = -10 * y_from - 6 * yd_from - 2.5 * ydd_from + 10 * y_to - 4 * yd_to + 0.5 * ydd_to
        a4 = 15 * y_from + 8 * yd_from + 2 * ydd_from - 15  * y_to  + 7 * yd_to - ydd_to
        a5 = -6 * y_from - 3 * yd_from - 0.5 * ydd_from  + 6 * y_to  - 3 * yd_to + 0.5 * ydd_to

        n_time_steps = ts.size
        n_dims = y_from.size
  
        ys = np.zeros([n_time_steps,n_dims])
        yds = np.zeros([n_time_steps,n_dims])
        ydds = np.zeros([n_time_steps,n_dims])

        for i in range(n_time_steps):
            t = (ts[i] - ts[0]) / (ts[n_time_steps - 1] - ts[0])
            ys[i,:] = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5)
            yds[i,:] = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4)
            ydds[i,:] = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3)

        yds /= (ts[n_time_steps - 1] - ts[0])
        ydds /= pow(ts[n_time_steps - 1] - ts[0], 2)
        #path = [(path_x[i], path_y[i]) for i in range(len(ts))] # SUGGESTE syntax for eventually polynomial path 
        return ts, ys, yds, ydds

