from math import pi
import numpy as np
# Simulation parameters
TIME_DURATION = 600 # (s)
TIME_STEP = 0.01
TIME_SCALER = 80 # TIME SCALER OF THE SIMULATION 
OPTIMIZATION_ON = True


# Estimation Parameters
TP = 40 # regressor MAX length

SIGMA_MEAS = 0.002# # uncertainty = 5° --> sigma^2 = (uncertainty*pi/180)^2


# Team parameter: number of agents, baselines_XY, inital position, type of formation
PLATFORM_INIT_POSE = [0, 0, 0] #[x,y,theta]
AUV_VEL = 1.0 #(m/s)#2 # nominal vel
AUV_MAX_VEL = 2.0 #(m/s)#4 # max vel considering v_coop
GAIN_YAW_RATE = 1.0#TODO: check the final tuning 

# Target parameter: start, goal, min max vels
# PARTE SEMPRE DA UNA DISTANZA COMPRESA TRA I 3.5 E 5 KM con velocità da 4 a 8 m/s

#TARGET_INIT = [+2000,-2500, pi, 2.5] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 1
#TARGET_INIT = [3000,-1500,pi/2,9.0]#[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 2
TARGET_INIT = [4000, 200, 140*pi/180, 8.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 3
#TARGET_INIT = [-3000, -2000, pi/2, 7.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 4
#TARGET_INIT = [-1500, 2000, pi/8, 5.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 5

#TARGET_INIT = [1300,0,140*pi/180,3.0] - SIMPLE CASE (lower distances)
#TARGET_INIT = [400,-200, pi/2, 3.0]
#TARGET_INIT = [20,0, pi/2, 1.0]
#TARGET_INIT = [-1000, -500, pi/2, 8.0] #[x(m),y(m),theta(rad),linear vel(m/s)]
TARGET_VEL = TARGET_INIT[3] #(m/s)
MAX_TARGET_VEL = 3 #(m/s) (only if target no costant vels)
MIN_TARGET_VEL = 3 #(m/s)

# Communication Paramaters (to generalize) - for now based on v-sense data
s = 1
mean = [s*5.0, s*2.5, s*1.5, 0.0]  #medium latencies between each AUV and the 4th (in fact latencies 0.0 for the 4th).
variance = [s*1.0, s*0.8, s*0.3, 0.0] #the same as before vor the variances.
MEAS_UPDATE = mean[0]/s# TODO# tune right this value
time_scaler = 5 # TIME SCALER OF THE SIMULATION INSIDE OPTIMIZATION

# Optimization Parameters
STATE_PROPAGATION = mean[0] #time between each propagation of the estimation (in real case a consensus (?))

k_max = 15*pi/180
delta_k = 5*pi/180
U = 3 #number of control choices
M = 3 # planning horizon
OPTIMIZATION_TIME_STEP = 40 #STATE_PROPAGATION*(TIME_SCALER*TIME_STEP) #VA INTESO COME delta_k (planning stage)in secondi

ctrl_cmd = [-k_max,0,+k_max]
#ctrl_cmd = [-k_max, -k_max*4/(U),0,k_max*4/(U),k_max] # simplified set of control actions for fast debugging
#ctrl_cmd = [-k_max, -k_max*4/(U),-k_max*2/(U),0,k_max*2/(U),k_max*4/(U),k_max] #set of control actions

# Cooperative Path Following Params
a, b = 1, -1
K_att = 1.0#0.0005# for in line -> 0.05 # Attractive gain #TODO da tunare per main e per opt diversamente
K_rep = 00.0 #1.0 #for in line -> 10.0 # Repulsive gain
d_rep = 150 # Distance threshold for repulsion
# CHOOSE THE GEOMETRY BETWEEN THE AGENTS
geometry = 'line'

if geometry == 'line':
    formation =  np.array([[0, 450],
                            [0,-150], 
                            [0, 150], 
                            [0,-450]]) # IN LINEA    
elif geometry == 'column':
    formation = [900,600,300,0] # IN COLONNA

elif geometry == 'polygon':
    formation =  np.array([ [170,-200],
                            [0, 100], 
                            [0,-100], 
                            [170,200]]) # TRAPEZOIDALE
elif geometry == 'line2':
    formation =  np.array([ [0, -25],
                            [0, +25]]) # TRAPEZOIDALE
elif geometry == 'column2':
    formation =  [50]
elif geometry == 'one_auv':
    formation =  np.array([[0, 0]]) # TRAPEZOIDALE 
N_AUV = len(formation)

class Pose:
    """2D pose"""

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

def calc_spline_course(sp,ds):
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))
    return rx, ry, ryaw, rk, s

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