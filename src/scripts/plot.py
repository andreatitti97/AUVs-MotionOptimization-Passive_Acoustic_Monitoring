from random import sample
from cvxpy import length
import matplotlib.pyplot as plt
from math import pi
import numpy as np
import sys, os
from main2 import TIME_DURATION
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs/plot')
sys.path.append(lib_path)

#LOAD LOG FILES
ekf_y = np.loadtxt(lib_path+'/target_est_y.txt')
real_y = np.loadtxt(lib_path+'/target_y_traj.txt')
ekf_x = np.loadtxt(lib_path+'/target_est_x.txt')
real_x = np.loadtxt(lib_path+'/target_x_traj.txt')
err_x = np.loadtxt(lib_path+'/rmse_y.txt')
err_y = np.loadtxt(lib_path+'/rmse_x.txt')
s_x = np.loadtxt(lib_path+'/x_platform.txt')
s_y = np.loadtxt(lib_path+'/y_platform.txt')

# PLOT
n_sample = np.size(ekf_y)
t = np.linspace(0,TIME_DURATION,n_sample)
plt.subplot(2,1,1)
plt.plot(t,ekf_y[0:n_sample],'g')
plt.plot(t,real_y[0:n_sample],'r--')
plt.xlabel('time(s)')
plt.ylabel('y pos target (m)')
plt.legend(['estimated','real'])
plt.grid()
plt.subplot(2,1,2)
plt.plot(t,ekf_x[0:n_sample],'g')
plt.plot(t,real_x[0:n_sample],'r--')

plt.xlabel('time (s)')
plt.ylabel('x pos target (m)')
plt.legend(['estimated','real'])
plt.grid()
plt.show()

plt.plot(s_x,s_y)
plt.plot(real_x,real_y)

plt.xlabel('x platform (m)')
plt.ylabel('y platform (m)')
circle1 = plt.Circle((3000,0),50,color='r')
circle2 = plt.Circle((real_x[0],real_x[0]),25,color='y')
circle3 = plt.Circle((s_x[0],s_y[0]),25,color='b')
plt.gca().add_patch(circle1)
plt.gca().add_patch(circle2)
plt.gca().add_patch(circle3)
plt.legend(['platform path','target path','base','target start','platform start'])
plt.grid()
plt.show()

plt.plot(t,err_x[0:n_sample])
plt.plot(t,err_y[0:n_sample])
plt.legend(['along x ','along y'])
plt.xlabel('time (s)')
plt.ylabel('RMSE')
plt.grid()
plt.show()

