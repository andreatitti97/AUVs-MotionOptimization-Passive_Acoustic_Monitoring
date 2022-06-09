import matplotlib.pyplot as plt
import numpy as np
import os
from main2 import TIME_DURATION
from math import pi
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')

#LOAD LOG FILES
# Estimation vs Real (in time)
ekf_y_on = np.loadtxt(lib_path+'/target_est_y_ON.txt')
ekf_x_on = np.loadtxt(lib_path+'/target_est_x_ON.txt')

real_y = np.loadtxt(lib_path+'/target_y_traj.txt')
real_x = np.loadtxt(lib_path+'/target_x_traj.txt')

ekf_y_off = np.loadtxt(lib_path+'/target_est_y_OFF.txt')
ekf_x_off = np.loadtxt(lib_path+'/target_est_x_OFF.txt')

# RMSE optimization ON vs OFF
err_y_on = np.loadtxt(lib_path+'/rmse_y_ON.txt')
err_x_on = np.loadtxt(lib_path+'/rmse_x_ON.txt')
err_on = np.loadtxt(lib_path+'/rmse_ON.txt')
err_y_off = np.loadtxt(lib_path+'/rmse_y_OFF.txt')
err_x_off = np.loadtxt(lib_path+'/rmse_x_OFF.txt')
err_off = np.loadtxt(lib_path+'/rmse_OFF.txt')
# LOAD bearing during sim
bearing1_on = np.loadtxt(lib_path+'/bearing1_ON.txt')
bearing2_on = np.loadtxt(lib_path+'/bearing2_ON.txt')
bearing1_off = np.loadtxt(lib_path+'/bearing1_OFF.txt')
bearing2_off = np.loadtxt(lib_path+'/bearing2_OFF.txt')
# Platform and AUV path OFF vs ON
s_x_on = np.loadtxt(lib_path+'/x_platform_ON.txt')
s_y_on = np.loadtxt(lib_path+'/y_platform_ON.txt')
auv1_x_on = np.loadtxt(lib_path+'/auv1_x_ON.txt')
auv1_y_on = np.loadtxt(lib_path+'/auv1_y_ON.txt')
auv2_x_on = np.loadtxt(lib_path+'/auv2_x_ON.txt')
auv2_y_on = np.loadtxt(lib_path+'/auv2_y_ON.txt')

s_x_off = np.loadtxt(lib_path+'/x_platform_OFF.txt')
s_y_off = np.loadtxt(lib_path+'/y_platform_OFF.txt')
auv1_x_off = np.loadtxt(lib_path+'/auv1_x_OFF.txt')
auv1_y_off = np.loadtxt(lib_path+'/auv1_y_OFF.txt')
auv2_x_off = np.loadtxt(lib_path+'/auv2_x_OFF.txt')
auv2_y_off = np.loadtxt(lib_path+'/auv2_y_OFF.txt')
# Optimization module data (ctrl_cmds, estimation ...) for DEBUGGING PURPOSE
opt_est_x = np.loadtxt(lib_path+'/target_traj_est_x.txt')
opt_est_y = np.loadtxt(lib_path+'/target_traj_est_y.txt')
opt_real_x = np.loadtxt(lib_path+'/target_traj_real_x.txt')
opt_real_y = np.loadtxt(lib_path+'/target_traj_real_y.txt')
ctrl_cmds = np.loadtxt(lib_path+'/plot_cmds.txt')

# Load temporal vaiables
n_sample = np.size(ekf_y_on)
t = np.linspace(0,TIME_DURATION,n_sample)
# PLOT THE RESULT OF THE SIMULATION without OPTIMIZATION
plt.plot(s_x_off,s_y_off)
plt.plot(real_x,real_y)

plt.plot(ekf_x_off,ekf_y_off)
plt.plot(auv1_x_off,auv1_y_off,'r',linewidth=2)
plt.plot(auv2_x_off,auv2_y_off,'g',linewidth=2)
plt.title('SIMULATION',fontsize=20)
plt.xlabel('x (m)',fontsize=20)
plt.ylabel('y (m)',fontsize=20)
circle1 = plt.Circle((2000,0),50,color='m')
circle2 = plt.Circle((real_x[0],real_y[0]),25,color='y')
circle3 = plt.Circle((s_x_off[0],s_y_off[0]),25,color='b')
circle4 = plt.Circle((auv1_x_off[0],auv1_y_off[0]),25,color='r')
circle5 = plt.Circle((auv2_x_off[0],auv2_y_off[0]),25,color='g')

plt.gca().add_patch(circle1)
plt.gca().add_patch(circle2)
plt.gca().add_patch(circle3)
plt.gca().add_patch(circle5)
plt.gca().add_patch(circle4)
plt.legend(['Formation Reference Path','Target Path','Estimated Target Path','AUV1 path','AUV2 path','Boat',
'Target Start','Formation Reference start','AUV1 start','AUV2 start'])
plt.grid()
plt.show()

# PLOT THE RESULT OF THE SIMULATION with OPTIMIZATION
plt.plot(s_x_on,s_y_on)
plt.plot(real_x,real_y)
plt.plot(ekf_x_on,ekf_y_on,'m')
plt.plot(auv1_x_on,auv1_y_on,'r',linewidth=2)
plt.plot(auv2_x_on,auv2_y_on,'g',linewidth=2)

plt.title('SIMULATION - OPTIMIZATION ON',fontsize=20)
plt.xlabel('x (m)',fontsize=20)
plt.ylabel('y (m)',fontsize=20)
circle1 = plt.Circle((2000,0),50,color='m')
circle2 = plt.Circle((real_x[0],real_y[0]),25,color='y')
circle3 = plt.Circle((s_x_on[0],s_y_on[0]),25,color='b')
circle4 = plt.Circle((auv1_x_on[0],auv1_y_on[0]),25,color='r')
circle5 = plt.Circle((auv2_x_on[0],auv2_y_on[0]),25,color='g')

plt.gca().add_patch(circle1)
plt.gca().add_patch(circle2)
plt.gca().add_patch(circle3)
plt.gca().add_patch(circle5)
plt.gca().add_patch(circle4)
plt.legend(['Formation Reference Path','Target Path','Estimated Target Path','AUV1 path','AUV2 path','Boat',
'Target Start','Formation Reference start','AUV1 start','AUV2 start'])
for i in range(7):
    
    plt.plot([auv1_x_on[n_sample-(i+1)*500],ekf_x_on[n_sample-(i+1)*500]],[auv1_y_on[n_sample-(i+1)*500],ekf_y_on[n_sample-(i+1)*500]],'grey')
    plt.plot([auv2_x_on[n_sample-(i+1)*500],ekf_x_on[n_sample-(i+1)*500]],[auv2_y_on[n_sample-(i+1)*500],ekf_y_on[n_sample-(i+1)*500]],'grey')
plt.grid()
plt.show()

bearing1_off = bearing1_off *180/pi
bearing2_off = bearing2_off *180/pi
bearing1_on = bearing1_on *180/pi
bearing2_on = bearing2_on *180/pi

bearing_diff = bearing1_on - bearing2_on
plt.subplot(3,1,1)
plt.title('RELATIVE BEARING MEASURED - OPTIMIZATION ON')
plt.plot(t,bearing1_on,'r')
plt.plot(t,bearing2_on,'g')
plt.ylabel('Relative Bearing (deg)')
y = np.zeros(n_sample)
for i in range(n_sample): y[i] = 60
plt.plot(t,y,'b--')
for i in range(n_sample): y[i] = 120
plt.plot(t,y,'b--')
plt.legend(['AUV1','AUV2'])
plt.grid()


plt.subplot(3,1,2)
plt.title('RELATIVE BEARING MEASURED - OPTIMIZATION OFF')
plt.plot(t,bearing1_off,'r')
plt.plot(t,bearing2_off,'g')
plt.ylabel('Relative Bearing (deg)')
for i in range(n_sample): y[i] = 60
plt.plot(t,y,'b--')
for i in range(n_sample): y[i] = 120
plt.plot(t,y,'b--')
plt.legend(['AUV1','AUV2'])
plt.grid()


plt.subplot(3,1,3)
plt.title('DIFFERENCE of BEARING MEASURED - OPTIMIZATION ON')
plt.plot(t,bearing_diff,'k',markerfacecolor='yellow')
for i in range(n_sample): y[i] = 0
plt.plot(t,y,'b--')
for i in range(n_sample): y[i] = 5
plt.plot(t,y,'b--')
for i in range(n_sample): y[i] = -5
plt.plot(t,y,'b--')
plt.ylabel('Difference-Relative Bearing(deg)')
plt.xlabel('Time (s)')
plt.legend(['Angle Difference'])
plt.grid()
plt.show()
# PLOT OPTIMIZATION TARGET PREDICTION 

plt.plot(opt_real_x,opt_real_y,'o')
plt.title(['TARGET PREDICTION DURING OPTIMIZATION'])
plt.xlabel('x pos target (m)',fontsize=20)
plt.ylabel('y pos target (m)',fontsize=20)
plt.legend(['estimated','real'])
plt.grid()
plt.show()

# PLOT ctrl cmds from optimization
n_sample1 = np.size(ctrl_cmds)
t1 = np.linspace(0,TIME_DURATION,n_sample1)
plt.plot(t1,ctrl_cmds,'-ok',markerfacecolor='blue')
plt.title('HEADING CHANGE COMMANDED')
plt.xlabel('time (s)',fontsize=20)
plt.ylabel('Heading Changes (rad)',fontsize=20)
plt.grid()
plt.show()

# COMPARE RMSE 
plt.plot(t,err_on[0:n_sample])
plt.plot(t,err_off[0:n_sample])
plt.legend(['optimization ON','optimization OFF'])
plt.title('ESTIMATION PERFORMANCES COMPARISON')
plt.xlabel('time (s)',fontsize=20)
plt.ylabel('RMSE (m)',fontsize=20)
plt.grid()
plt.show()


