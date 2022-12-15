from math import pi
import numpy as np
# Simulation parameters
TIME_DURATION = 2800 # (s) c.a. 45 min
TIME_STEP = 0.01
TIME_SCALER = 80 # MAX for communication purpose 
TARGET_INIT = [8000,  8000, pi-pi/10] #[x(m),y(m),theta(rad),linear vel(m/s)]
TARGET_GOAL = [-12000, 12000,TARGET_INIT[2]]
PLATFORM_INIT_POSE = [1000, 1000, 0] #[x,y,theta]
SIGMA_MEAS = 0.01 # uncertainty = 1° --> sigma^2 = (uncertainty*2*pi/180)^2  per ora 3 gradi
ALONG_BAR_FORMATION = True
OPTIMIZATION_ON = True
OPTIMIZATION_TIME_STEP = 64 #VA INTESO COME time between each command 
TIME_COUNTER = (OPTIMIZATION_TIME_STEP/(TIME_STEP*TIME_SCALER))
BASELINE_Y = 3000 #
BASELINE_X = 1000 #valori presi dall'esperimento sulla comunicazione (veicoli lontani)
N_AUV = 4
MAX_TARGET_VEL = 10 #(m/s)
MIN_TARGET_VEL = 3 #(m/s)
MEAS_UPDATE = 15
STATE_PROPAGATION = 15

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

        return ts, ys, yds, ydds
