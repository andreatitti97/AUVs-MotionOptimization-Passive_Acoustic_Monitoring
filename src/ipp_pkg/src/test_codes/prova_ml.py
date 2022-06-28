import numpy as np
from scipy.optimize import minimize
import scipy.stats as stats
import pandas as pd
import statsmodels. api as sm
import matplotlib.pyplot as plt
import os
utils_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/utils')
plot_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')

m = 0.0
b = 1.0
sigma = 0.1
data = np.loadtxt(plot_path+'/target_est_y_ON.txt')
gt = np.loadtxt(plot_path+'/target_y_traj.txt')
#y = np.loadtxt(plot_path+'/target_traj_real_x.txt')
n_samples = np.size(data)
t = np.linspace(0,600,n_samples)
y_exp = gt
print(y_exp[100],data[100])
tmp = y_exp[100] - data[100]
L = np.sum(np.log(stats.norm.pdf(tmp, loc = m, scale=sigma)))
print(L)