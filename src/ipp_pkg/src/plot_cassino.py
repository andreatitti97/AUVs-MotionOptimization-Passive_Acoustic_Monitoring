import matplotlib.pyplot as plt
import numpy as np
import os
from main import BASELINE_X, TIME_DURATION, N_AUV, BASELINE_Y
from math import pi
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')

#LOAD LOG FILES
# Estimation vs Real (in time)

real_y = np.loadtxt(lib_path+'/target_y_traj.txt')
real_x = np.loadtxt(lib_path+'/target_x_traj.txt')

est1_x_OFF = np.loadtxt(lib_path+'/est1_x_OFF.txt')
est1_y_OFF = np.loadtxt(lib_path+'/est1_y_OFF.txt')
est2_x_OFF = np.loadtxt(lib_path+'/est2_x_OFF.txt')
est2_y_OFF = np.loadtxt(lib_path+'/est2_y_OFF.txt')
if N_AUV > 2:
    est3_x_OFF = np.loadtxt(lib_path+'/est3_x_OFF.txt')
    est3_y_OFF = np.loadtxt(lib_path+'/est3_y_OFF.txt')
    est4_x_OFF = np.loadtxt(lib_path+'/est4_x_OFF.txt')
    est4_y_OFF = np.loadtxt(lib_path+'/est4_y_OFF.txt')

# RMSE optimization ON vs OFF

err_on = np.loadtxt(lib_path+'/rmse_ON.txt')
err_off = np.loadtxt(lib_path+'/rmse_OFF.txt')

# Platform and AUV path OFF vs ON

s_x_off = np.loadtxt(lib_path+'/x_platform_OFF.txt')
s_y_off = np.loadtxt(lib_path+'/y_platform_OFF.txt')
auv1_x_off = np.loadtxt(lib_path+'/auv1_x_OFF.txt')
auv1_y_off = np.loadtxt(lib_path+'/auv1_y_OFF.txt')
auv2_x_off = np.loadtxt(lib_path+'/auv2_x_OFF.txt')
auv2_y_off = np.loadtxt(lib_path+'/auv2_y_OFF.txt')
if N_AUV > 2:
    auv3_x_off = np.loadtxt(lib_path+'/auv3_x_OFF.txt')
    auv3_y_off = np.loadtxt(lib_path+'/auv3_y_OFF.txt')
    auv4_x_off = np.loadtxt(lib_path+'/auv4_x_OFF.txt')
    auv4_y_off = np.loadtxt(lib_path+'/auv4_y_OFF.txt')

# Load temporal vaiables
n_sample = np.size(est1_x_OFF)
t = np.linspace(0,TIME_DURATION,n_sample)
# PLOT THE RESULT OF THE SIMULATION without OPTIMIZATION
plt.plot(s_x_off,s_y_off)
plt.plot(real_x,real_y)
plt.plot(est1_x_OFF,est1_y_OFF,'r')
plt.plot(est2_x_OFF,est2_y_OFF,'g')
if N_AUV > 2:
    plt.plot(est3_x_OFF,est3_y_OFF,'m')
    plt.plot(est4_x_OFF,est4_y_OFF,'k')
plt.title('SIMULATION - OPTIMIZATION OFF',fontsize=20)
plt.xlabel('x (m)',fontsize=20)
plt.ylabel('y (m)',fontsize=20)
circle1 = plt.Circle((2000,0),50,color='grey')
circle2 = plt.Circle((real_x[0],real_y[0]),100,color='y')
circle3 = plt.Circle((s_x_off[0],s_y_off[0]),100,color='b')
circle4 = plt.Circle((auv1_x_off[0],auv1_y_off[0]),100,color='r')
circle5 = plt.Circle((auv2_x_off[0],auv2_y_off[0]),100,color='g')
plt.gca().add_patch(circle1)
plt.gca().add_patch(circle2)
plt.gca().add_patch(circle3)
plt.gca().add_patch(circle4)
plt.gca().add_patch(circle5)
if N_AUV > 2:
    circle6 = plt.Circle((auv3_x_off[0],auv3_y_off[0]),100,color='m')
    circle7 = plt.Circle((auv4_x_off[0],auv4_y_off[0]),100,color='k')
    plt.gca().add_patch(circle6)
    plt.gca().add_patch(circle7)




if N_AUV > 2:
    plt.legend(['Formation Reference Path','Target Path','AUV1-Target Estimation',
    'AUV2-Target Estimation','AUV3-Target Estimation','AUV4-Target Estimation','OPERATOR LOCATION',
    'Target Start','Formation Reference','AUV1','AUV2','AUV3','AUV4'])
else:
    plt.legend(['Formation Reference Path','Target Path','AUV1-Target Estimation',
    'AUV2-Target Estimation','OPERATOR LOCATION',
    'Target Start','Formation Reference','AUV1','AUV2'])

for i in range(1):
    
    plt.plot([auv1_x_off[n_sample-(i+1)*500],
        est1_x_OFF[n_sample-(i+1)*500]],[auv1_y_off[n_sample-(i+1)*500],est1_y_OFF[n_sample-(i+1)*500]],'k--',linewidth=0.5)
    plt.plot([auv2_x_off[n_sample-(i+1)*500],est1_x_OFF[n_sample-(i+1)*500]],[auv2_y_off[n_sample-(i+1)*500],
        est1_y_OFF[n_sample-(i+1)*500]],'k--',linewidth=0.5)
    plt.plot(auv1_x_off[n_sample-(i+1)*500],auv1_y_off[n_sample-(i+1)*500],'or')
    plt.plot(auv2_x_off[n_sample-(i+1)*500],auv2_y_off[n_sample-(i+1)*500],'og')
    if N_AUV > 2:
        plt.plot(auv3_x_off[n_sample-(i+1)*500],auv3_y_off[n_sample-(i+1)*500],'om')
        plt.plot(auv4_x_off[n_sample-(i+1)*500],auv4_y_off[n_sample-(i+1)*500],'ok')
    plt.plot(s_x_off[n_sample-(i+1)*500],s_y_off[n_sample-(i+1)*500],'ob') 
plt.axis('equal')
plt.grid()
plt.show()

