from math import pi
import numpy as np
import os
import importlib.util
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.utils", class_path+"/utils.py")
utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(utils)
############################################################ SIMULATION SETUP ########################################################
# Simulation parameters
TIME_DURATION = 1200 # (s)
TIME_STEP = 0.01
TIME_SCALER = 80 # TIME SCALER OF THE SIMULATION 
c = 1500 #sound wave speed
OPTIMIZATION_ON = True
# Estimation Parameters
TP = 30 # regressor MAX length 40
SIGMA_MEAS = 0.08#0.02 #(rad^2) --> 4.5° (as assumed in DAMPS and by cassino)
# Team parameter: number of agents, baselines_XY, inital position, type of formation
PLATFORM_INIT_POSE = [0, 0, 0] #[x,y,theta]
AUV_VEL = 2.0 #(m/s)#2 # nominal vel 
AUV_MAX_VEL = 6.0 #(m/s)#4 # max vel considering v_coop 3.0
GAIN_YAW_RATE = 0.8 
# CHOOSE THE GEOMETRY BETWEEN THE AGENTS
geometry = 'line'
# Communication Paramaters
d = 450 #vehicle distance 300
Tm = 5 # measurements time sampling #3 IDEAL CASE
Tg = 3 # time slot for each vehicle #2 IDEAL CASE 
# Optimization Parameters
time_scaler = 5 # TIME SCALER OF THE SIMULATION INSIDE OPTIMIZATION
u_max = 20*pi/180
delta_u = 3*pi/180
MAX = 40*pi/180
MIN = 5*pi/180
U = 7 #number of control choices
M = 3 # planning horizon
#ctrl_cmd = [-u_max,0,+u_max]
#ctrl_cmd = [-u_max, -u_max*4/(U),0,u_max*4/(U),u_max] # simplified set of control actions for fast debugging
ctrl_cmd = [-u_max, -u_max*4/(U),-u_max*2/(U),0,u_max*2/(U),u_max*4/(U),u_max] #set of control actions
##################################################################################################################################


######## CHOOSE TARGET DYNAMIC ###################################################################################################
# CHOOSE Target parameter: start, goal, min max vels
# PARTE SEMPRE DA UNA DISTANZA COMPRESA TRA I 3.5 E 5 KM con velocità da 4 a 8 m/s
#TARGET_INIT = [400,-200, pi/2, 3.0, 0.0, 0.0, 0.0] #SIMPLE CASE LOWE DISTANCE!!!!!

TARGET_INIT = [+2000,-2500, pi, 2.5, 0.0, 0.0, 0.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 1
#TARGET_INIT = [4000, 200, 140*pi/180, 8.0, 0.0, 0.0, 0.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 2
#TARGET_INIT = [-5000,-5000, pi/4, 3.0, 0.0, 0.0, 0.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 3
#TARGET_INIT = [-500,+3500, -pi/10, 2.5, 0.0, 0.0, 0.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 4
#TARGET_INIT = [3000,-1500, pi/2, 9.0, 0.0, 0.5, 0.0]#[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 5
#TARGET_INIT = [-2000, +2000, 140*pi/180, 8.0, 0.002, 0.0, 0.0] #[x(m),y(m),theta(rad),v0(m/s),omega0,v_dot0,omega_dot0] - DINAMICA 6
#TARGET_INIT = [-2000, -1800, pi/2, 6.0, -0.001, 0.0, 0.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 7
#TARGET_INIT = [-1500, 2000, pi/8, 5.0, 0.0, 0.8, 0.0] #[x(m),y(m),theta(rad),linear vel(m/s)] - DINAMICA 8

alpha_0, omega_0,alpha_dot_0,omega_dot_0 = TARGET_INIT[3],TARGET_INIT[4],TARGET_INIT[5],TARGET_INIT[6]
MAX_TARGET_VEL = 3 #(m/s) (only if target no costant vels)
MIN_TARGET_VEL = 3 #(m/s)
###################################################################################################################################

# DO NOT EDIT ##############################################################
formation, mean, variance = utils.generate_formation(geometry,d,Tg,Tm,c)

N_AUV = len(formation)
# Cooperative Path Following Params
K_att = 0.5 # Attractive Gain
K_rep = 0.0 # Repulsive gain
d_rep = d # Distance threshold for repulsion
Tf = 2*N_AUV*Tg #time frame TDMA

#OPTIMIZATION_TIME_STEP = 20
OPTIMIZATION_TIME_STEP = int(np.ceil(2*(Tm*N_AUV+np.sum(mean))))#TODO





