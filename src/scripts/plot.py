from cvxpy import length
import matplotlib.pyplot as plt
from math import pi
import numpy as np
import sys, os
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs/plot')
sys.path.append(lib_path)

ekf_y = np.loadtxt(lib_path+'/target_est_y.txt')
real_y = np.loadtxt(lib_path+'/target_y_traj.txt')
ekf_x = np.loadtxt(lib_path+'/target_est_x.txt')
real_x = np.loadtxt(lib_path+'/target_x_traj.txt')
err_x = np.loadtxt(lib_path+'/rmse_y.txt')
err_y = np.loadtxt(lib_path+'/rmse_x.txt')
T = np.size(ekf_y)
dt = T/0.01
print(T)
t = np.linspace(0,dt,T)
plt.subplot(2,1,1)
plt.plot(t,ekf_y[0:T])
plt.plot(t,real_y[0:T])
plt.xlabel('TIME')
plt.ylabel('y_real vs y_ekf')
plt.subplot(2,1,2)
plt.plot(t,ekf_x[0:T])
plt.plot(t,real_x[0:T])
plt.xlabel('TIME')
plt.ylabel('x_real vs x_ekf')
plt.show()

plt.plot(t,err_x[0:T])

plt.plot(t,err_y[0:T])
plt.show()