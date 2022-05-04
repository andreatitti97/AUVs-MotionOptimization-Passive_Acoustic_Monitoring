from cvxpy import length
import matplotlib.pyplot as plt
from math import pi
import numpy as np
import sys, os
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs')
sys.path.append(lib_path)

ekf_y = np.loadtxt(lib_path+'/ekf_out_y.txt')
real_y = np.loadtxt(lib_path+'/target_y_traj.txt')
err = np.loadtxt(lib_path+'/err_medio.txt')
T = np.size(ekf_y)
dt = T/0.01
print(T)
t = np.linspace(0,dt,T)
plt.plot(t,ekf_y)
plt.plot(t,real_y[0:-1])
plt.xlabel('TIME')
plt.ylabel('y_real vs y_ekf')
plt.show()

plt.plot(t,err[0:-1])
plt.show()