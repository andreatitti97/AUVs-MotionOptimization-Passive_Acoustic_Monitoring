from math import pi
import numpy as np
# Simulation parameters
TIME_DURATION = 600 # (s)
TIME_STEP = 0.01
TIME_SCALER = 80 # MAX for communication purpose 
OPTIMIZATION_ON = True

# Estimation Parameters
TP = 50 # regressor MAX length
STATE_PROPAGATION = 8 #time between each propagation of the estimation (in real case a consensus (?))
# Team parameter: number of agents, baselines_XY, inital position, type of formation
SIGMA_MEAS = 0.02 #0.005 # uncertainty = 1° --> sigma^2 = (uncertainty*2*pi/180)^2  per ora 3 gradi -- TO DECIDE!!
MEAS_UPDATE = STATE_PROPAGATION/4 # MEAS_UPDATE (s)= meas_update*(TIME_SCALER*TIME_STEP)
N_AUV = 4
BASELINE_Y = 100 #(m)
BASELINE_X = 0 #valori presi dall'esperimento sulla comunicazione (veicoli lontani)
PLATFORM_INIT_POSE = [0, 0, 0] #[x,y,theta]

AUV_VEL = 1.0 #(m/s)
CONTROLLER_GAIN = 0.5
# Target parameter: start, goal, min max vels
TARGET_INIT = [1200, -500, pi/2] #[x(m),y(m),theta(rad),linear vel(m/s)]
TARGET_GOAL = [100, 100,TARGET_INIT[2]]
TARGET_VEL = 3 #(m/s)
MAX_TARGET_VEL = 3 #(m/s) (only for no costant vels)
MIN_TARGET_VEL = 3 #(m/s)

formation = np.array([[PLATFORM_INIT_POSE[0], PLATFORM_INIT_POSE[1]],[0, -50], 
                            [0,50], 
                            [50,-180], 
                            [50,180]])


formation = np.array([[PLATFORM_INIT_POSE[0], PLATFORM_INIT_POSE[1]],[0, -50], 
                            [0,50], 
                            [0,-100], 
                            [0, 100]])

# Optimization Parameters
OPTIMIZATION_TIME_STEP = STATE_PROPAGATION*(TIME_SCALER*TIME_STEP) #VA INTESO COME delta_k (planning stage)in secondi
k_max = 15*pi/180 
delta_k = 0#1.5*pi/180 
U = 7 #number of control choices
M = 3 # planning horizon
ctrl_cmd = [-k_max, -k_max*4/(U),-k_max*2/(U),0,k_max*2/(U),k_max*4/(U),k_max]

# Cooperative Path Following Params
K_att = 1.0 # Attractive gain
K_rep = 0.0 # Repulsive gain
d_rep = 0.2 # Distance threshold for repulsion


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

