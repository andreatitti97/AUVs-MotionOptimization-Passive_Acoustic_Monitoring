from math import pi
import numpy as np
# Simulation parameters
TIME_DURATION = 600 # (s)
TIME_STEP = 0.01
TIME_SCALER = 80 # TIME SCALER OF THE SIMULATION 

OPTIMIZATION_ON = True


# Estimation Parameters
TP = 30 # regressor MAX length 40
SIGMA_MEAS = 0.005#0.02# # uncertainty = 5° --> sigma^2 = (uncertainty*pi/180)^2,  0.05

# Team parameter: number of agents, baselines_XY, inital position, type of formation
PLATFORM_INIT_POSE = [0, 0, 0] #[x,y,theta]
AUV_VEL = 1.0 #(m/s)#2 # nominal vel 
AUV_MAX_VEL = 3.0 #(m/s)#4 # max vel considering v_coop
GAIN_YAW_RATE = 0.8 
c = 1500 #sound wave speed
# Communication Paramaters

d = 300 #vehicle distance
Tm = 5 # measurements time sampling
Tg = 3 # time slot for each vehicle

# Optimization Parameters
time_scaler = 5 # TIME SCALER OF THE SIMULATION INSIDE OPTIMIZATION
u_max = 15*pi/180
delta_u = 3*pi/180
MAX = 40*pi/180
MIN = 5*pi/180
U = 5 #number of control choices
M = 3 # planning horizon
#ctrl_cmd = [-u_max,0,+u_max]
#ctrl_cmd = [-u_max, -u_max*4/(U),0,u_max*4/(U),u_max] # simplified set of control actions for fast debugging
ctrl_cmd = [-u_max, -u_max*4/(U),-u_max*2/(U),0,u_max*2/(U),u_max*4/(U),u_max] #set of control actions

# CHOSE Target parameter: start, goal, min max vels
# PARTE SEMPRE DA UNA DISTANZA COMPRESA TRA I 3.5 E 5 KM con velocità da 4 a 8 m/s

#TARGET_INIT = [+2000,-2500, pi, 2.5] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 1
#TARGET_INIT = [3000,-1500,pi/2,9.0]#[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 2
TARGET_INIT = [4000, 200, 140*pi/180, 8.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 3
#TARGET_INIT = [-3000, -2000, pi/2, 7.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 4
#TARGET_INIT = [-1500, 2000, pi/8, 5.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 5


#TARGET_INIT = [400,-200, pi/2, 1.5] --- SIMPLE CASE LOWE DISTANCE!!!!!

TARGET_VEL = TARGET_INIT[3] #(m/s)
MAX_TARGET_VEL = 3 #(m/s) (only if target no costant vels)
MIN_TARGET_VEL = 3 #(m/s)
# Cooperative Path Following Params
K_att = 0.5 # Attractive Gain
K_rep = 0.3 # Repulsive gain
d_rep = d # Distance threshold for repulsion
# CHOOSE THE GEOMETRY BETWEEN THE AGENTS
geometry = 'line'


if geometry == 'line':
    formation =  np.array([[0, d+(d/2)],
                            [0, +d/2], 
                            [0, -d/2], 
                            [0,-d-(d/2)]]) # IN LINEA  
    mean = [Tg+6*d/c,Tg+4*d/c,Tg+2*d/c,0]  #medium latencies between each AUV and the 4th (in fact latencies 0.0 for the 4th).
    variance = [Tm*1.0, Tm*0.8, Tm*0.3, 0.0] #the same as before vor the variances.
elif geometry == 'line2':
    formation =  np.array([ [0, +d],
                            [0, -d],
                            [0, +d],
                            [0, -d]]) # TRAPEZOIDALE
    mean = [d/c,d/c,0,0]  
    variance = [Tm*1.0, Tm*0.8, Tm*0.3, 0.0]
elif geometry == 'column':
    formation = [d*3,d*2,d*1,0] # IN COLONNA
    mean = [3*d/c,2*d/c,d/c,0]  
    variance = [Tm*1.0, Tm*0.8, Tm*0.3, 0.0]
elif geometry == 'polygon':
    formation =  np.array([ [0, +d/2],
                            [-d/2, 0],
                            [+d/2,0],
                            [0, -d/2]]) # TRAPEZOIDALE
    mean = [2*d/c,2*d/c,2*d/c,0] 
    variance = [Tm*1.0, Tm*0.8, Tm*0.3, 0.0]
elif geometry == 'column2':
    formation =  [d,d/2,d,d/2]
elif geometry == 'one_auv':
    formation =  np.array([[0, 0]]) # TRAPEZOIDALE 
N_AUV = len(formation)
Tf = N_AUV*Tg #time frame TDMA

OPTIMIZATION_TIME_STEP = np.sum(mean)

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
        #rk.append(sp.calc_curvature(i_s))
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