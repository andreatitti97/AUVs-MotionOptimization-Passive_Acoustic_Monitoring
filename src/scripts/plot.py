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
s_x = np.loadtxt(lib_path+'/y_platform.txt')
s_y = np.loadtxt(lib_path+'/x_platform.txt')
T = np.size(ekf_y)
dt = T/0.01
print(T)
t = np.linspace(0,dt,T)/10000
plt.subplot(2,1,1)
plt.plot(t,ekf_y[0:T])
plt.plot(t,real_y[0:T])
plt.xlabel('time(s)')
plt.ylabel('y pos target (m)')
#plt.xlim([-3,57])
#plt.ylim([-3,400])
plt.legend(['estimated','real'])
plt.grid()
plt.subplot(2,1,2)
plt.plot(t,ekf_x[0:T])
plt.plot(t,real_x[0:T])
#plt.xlim([-3,57])
#plt.ylim([-3,400])
plt.xlabel('time (s)')
plt.ylabel('x pos target (m)')
plt.legend(['estimated','real'])
plt.grid()
plt.show()

plt.plot(s_x,s_y)
plt.xlabel('x platform (m)')
plt.ylabel('y platform (m)')
plt.legend(['path'])
plt.grid()
plt.show()

plt.plot(t,err_x[0:T])
plt.plot(t,err_y[0:T])

#plt.xlim([-3,57])
#plt.ylim([-3,30])
plt.legend(['x ','y'])
plt.xlabel('time (s)')
plt.ylabel('RMSE')
plt.grid()
plt.show()

