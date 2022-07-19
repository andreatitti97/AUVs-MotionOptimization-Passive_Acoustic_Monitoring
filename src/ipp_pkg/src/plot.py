import matplotlib.pyplot as plt
import numpy as np
import os
from main import TIME_DURATION, N_AUV
from math import pi
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')

#LOAD LOG FILES
# Estimation vs Real (in time)
est1_x_ON = np.loadtxt(lib_path+'/est1_x_ON.txt')
est1_y_ON = np.loadtxt(lib_path+'/est1_y_ON.txt')
est2_x_ON = np.loadtxt(lib_path+'/est2_x_ON.txt')
est2_y_ON = np.loadtxt(lib_path+'/est2_y_ON.txt')
if N_AUV > 2:
    est3_x_ON = np.loadtxt(lib_path+'/est3_x_ON.txt')
    est3_y_ON = np.loadtxt(lib_path+'/est3_y_ON.txt')
    est4_x_ON = np.loadtxt(lib_path+'/est4_x_ON.txt')
    est4_y_ON = np.loadtxt(lib_path+'/est4_y_ON.txt')

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
s_x_on = np.loadtxt(lib_path+'/x_platform_ON.txt')
s_y_on = np.loadtxt(lib_path+'/y_platform_ON.txt')
auv1_x_on = np.loadtxt(lib_path+'/auv1_x_ON.txt')
auv1_y_on = np.loadtxt(lib_path+'/auv1_y_ON.txt')
auv2_x_on = np.loadtxt(lib_path+'/auv2_x_ON.txt')
auv2_y_on = np.loadtxt(lib_path+'/auv2_y_ON.txt')
if N_AUV > 2:
    auv3_x_on = np.loadtxt(lib_path+'/auv3_x_ON.txt')
    auv3_y_on = np.loadtxt(lib_path+'/auv3_y_ON.txt')
    auv4_x_on = np.loadtxt(lib_path+'/auv4_x_ON.txt')
    auv4_y_on = np.loadtxt(lib_path+'/auv4_y_ON.txt')

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
# Optimization CTRL_CMDS
ctrl_cmds = np.loadtxt(lib_path+'/plot_cmds.txt')
opt_x = np.loadtxt(lib_path+'/t_est_x_opt.txt')
opt_y = np.loadtxt(lib_path+'/t_est_y_opt.txt')
# Load temporal vaiables
n_sample = np.size(est1_x_ON)
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
if N_AUV > 2:
    circle6 = plt.Circle((auv3_x_off[0],auv3_y_off[0]),100,color='m')
    circle7 = plt.Circle((auv4_x_off[0],auv4_y_off[0]),100,color='k')
    plt.gca().add_patch(circle6)
    plt.gca().add_patch(circle7)

plt.gca().add_patch(circle1)
plt.gca().add_patch(circle2)
plt.gca().add_patch(circle3)
plt.gca().add_patch(circle4)
plt.gca().add_patch(circle5)


if N_AUV > 2:
    plt.legend(['Formation Reference Path','Target Path','AUV1-Target Estimation',
    'AUV2-Target Estimation','AUV3-Target Estimation','AUV3-Target Estimation','OPERATOR LOCATION',
    'Target Start','Formation Reference','AUV1','AUV2','AUV3','AUV4'])
else:
    plt.legend(['Formation Reference Path','Target Path','AUV1-Target Estimation',
    'AUV2-Target Estimation','OPERATOR LOCATION',
    'Target Start','Formation Reference','AUV1','AUV2'])

for i in range(5):
    
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

# PLOT THE RESULT OF THE SIMULATION with OPTIMIZATION
plt.plot(s_x_on,s_y_on)
plt.plot(real_x,real_y)
plt.plot(est1_x_ON,est1_y_ON,'r')
plt.plot(est2_x_ON,est2_y_ON,'g')
if N_AUV > 2:
    plt.plot(est3_x_ON,est3_y_ON,'m')
    plt.plot(est4_x_ON,est4_y_ON,'k')
plt.title('SIMULATION - OPTIMIZATION ON',fontsize=20)
plt.xlabel('x (m)',fontsize=20)
plt.ylabel('y (m)',fontsize=20)
circle1 = plt.Circle((2000,0),50,color='grey')
circle2 = plt.Circle((real_x[0],real_y[0]),100,color='y')
circle3 = plt.Circle((s_x_on[0],s_y_on[0]),100,color='b')
circle4 = plt.Circle((auv1_x_on[0],auv1_y_on[0]),100,color='r')
circle5 = plt.Circle((auv2_x_on[0],auv2_y_on[0]),100,color='g')
if N_AUV > 2:
    circle6 = plt.Circle((auv3_x_on[0],auv3_y_on[0]),100,color='m')
    circle7 = plt.Circle((auv4_x_on[0],auv4_y_on[0]),100,color='k')
    plt.gca().add_patch(circle6)
    plt.gca().add_patch(circle7)

plt.gca().add_patch(circle1)
plt.gca().add_patch(circle2)
plt.gca().add_patch(circle3)
plt.gca().add_patch(circle4)
plt.gca().add_patch(circle5)

if N_AUV > 2:
    plt.legend(['Formation Reference Path','Target Path','AUV1-Target Estimation',
    'AUV2-Target Estimation','AUV3-Target Estimation','AUV3-Target Estimation','OPERATOR LOCATION',
    'Target Start','Formation Reference','AUV1','AUV2','AUV3','AUV4'])
else:
    plt.legend(['Formation Reference Path','Target Path','AUV1-Target Estimation',
    'AUV2-Target Estimation','OPERATOR LOCATION',
    'Target Start','Formation Reference','AUV1','AUV2'])
for i in range(5):
    
    plt.plot([auv1_x_on[n_sample-(i+1)*500],
        est1_x_ON[n_sample-(i+1)*500]],[auv1_y_on[n_sample-(i+1)*500],est1_y_ON[n_sample-(i+1)*500]],'k--',linewidth=0.5)
    plt.plot([auv2_x_on[n_sample-(i+1)*500],est1_x_ON[n_sample-(i+1)*500]],[auv2_y_on[n_sample-(i+1)*500],
        est1_y_ON[n_sample-(i+1)*500]],'k--',linewidth=0.5)
    plt.plot(auv1_x_on[n_sample-(i+1)*500],auv1_y_on[n_sample-(i+1)*500],'or')
    plt.plot(auv2_x_on[n_sample-(i+1)*500],auv2_y_on[n_sample-(i+1)*500],'og')
    if N_AUV > 2:
        plt.plot(auv3_x_on[n_sample-(i+1)*500],auv3_y_on[n_sample-(i+1)*500],'om')
        plt.plot(auv4_x_on[n_sample-(i+1)*500],auv4_y_on[n_sample-(i+1)*500],'ok')
    plt.plot(s_x_on[n_sample-(i+1)*500],s_y_on[n_sample-(i+1)*500],'ob')  
plt.axis('equal')
plt.grid()
plt.show()

baseline_angle = []
for i in range(len(auv1_x_on)):
    tmp = (((real_x[i]-auv1_x_on[i])*(real_x[i]-auv2_x_on[i]))+((real_y[i]-auv1_y_on[i])*(real_y[i]-auv2_y_on[i]))
    )/(np.sqrt((real_x[i]-auv1_x_on[i])**2+(real_y[i]-auv1_y_on[i])**2)*np.sqrt((real_x[i]-auv2_x_on[i])**2+(
        real_y[i]-auv2_y_on[i])**2))

    angle = np.arccos(tmp)
    angle = angle*180/pi
    baseline_angle.append(angle)


baseline_angle_off = []
for i in range(len(auv1_x_off)):
    tmp = (((real_x[i]-auv1_x_off[i])*(real_x[i]-auv2_x_off[i]))+((real_y[i]-auv1_y_off[i])*(real_y[i]-auv2_y_off[i]))
    )/(np.sqrt((real_x[i]-auv1_x_off[i])**2+(real_y[i]-auv1_y_off[i])**2)*np.sqrt((real_x[i]-auv2_x_off[i])**2+(
        real_y[i]-auv2_y_off[i])**2))

    angle = np.arccos(tmp)
    angle = angle*180/pi
    baseline_angle_off.append(angle)

y = np.zeros(n_sample)
plt.subplot(2,1,1)

plt.title('OPTIMIZATION ON',fontsize=15)
plt.plot(t,baseline_angle,'k',markerfacecolor='yellow')
plt.ylabel('Angle FORMATION-TARGET(deg)',fontsize=15)
for i in range(n_sample): y[i] = 0
plt.plot(t,y,'r--',linewidth=2)
for i in range(n_sample): y[i] = 5
plt.plot(t,y,'y--',linewidth=2)
for i in range(n_sample): y[i] = 10
plt.plot(t,y,'g--',linewidth=2)

for i in range(n_sample): y[i] = -5
plt.plot(t,y,'y--',linewidth=2)

for i in range(n_sample): y[i] = -10
plt.plot(t,y,'g--',linewidth=2)
plt.legend(['Angle','No Baseline','Critical Baseline','Good Baseline'])

plt.grid()

plt.subplot(2,1,2)
plt.title('OPTIMIZATION OFF',fontsize=15)
plt.plot(t,baseline_angle_off,'k',markerfacecolor='yellow')
plt.ylabel('Angle FORMATION-TARGET(deg)',fontsize=15)
for i in range(n_sample): y[i] = 0
plt.plot(t,y,'r--',linewidth=2)
for i in range(n_sample): y[i] = 5
plt.plot(t,y,'y--',linewidth=2)
for i in range(n_sample): y[i] = 10
plt.plot(t,y,'g--',linewidth=2)
for i in range(n_sample): y[i] = -5
plt.plot(t,y,'y--',linewidth=2)

for i in range(n_sample): y[i] = -10
plt.plot(t,y,'g--',linewidth=2)

plt.xlabel('Time (s)',fontsize=20)
plt.legend(['Angle','No Baseline','Critical Baseline','Good Baseline'])
plt.grid()
plt.show()

# PLOT ctrl cmds from optimization
n_sample1 = np.size(ctrl_cmds)
t1 = np.linspace(640,TIME_DURATION,n_sample1)
plt.plot(t1,ctrl_cmds*180/pi,'-ok',markerfacecolor='blue')
plt.title('HEADING CHANGE COMMANDED',fontsize=20)
plt.xlabel('Time (s)',fontsize=20)
plt.ylabel('Heading Changes (deg)',fontsize=20)
plt.grid()
plt.show()

sum1 = 0
sum2 = 0
for i in range(len(err_off)):
    tmp1 = err_off[i]
    sum1 += tmp1
    tmp2 = err_on[i]
    sum2 += tmp2
err_medio1 = sum1/n_sample
err_medio2 = sum2/n_sample
print('ERRORE MEDIO OFF:',err_medio1)
print('ERRORE MEDIO ON:',err_medio2)
# COMPARE RMSE 
plt.plot(t,err_off[0:n_sample],'y')
plt.plot(t,err_on[0:n_sample],'b')
plt.legend(['optimization OFF','optimization ON'])
plt.title('ESTIMATION PERFORMANCES COMPARISON')
plt.xlabel('Time (s)',fontsize=20)
plt.ylabel('RMSE (m)',fontsize=20)
plt.text(t[300], err_off[300]+100, 'ERRORE MEDIO OFF:'+str(err_medio1), fontsize=15, color='y')
plt.text(t[300], err_off[300]+200, 'ERRORE MEDIO ON:'+str(err_medio2), fontsize=15, color='b')
plt.grid()
plt.show()

plt.plot(real_x, real_y)
plt.plot(opt_x,opt_y)
plt.grid()
plt.show()

# LOAD bearing during sim
#bearing1_on = np.loadtxt(lib_path+'/bearing1_ON.txt')
#bearing2_on = np.loadtxt(lib_path+'/bearing2_ON.txt')
#bearing1_off = np.loadtxt(lib_path+'/bearing1_OFF.txt')
#bearing2_off = np.loadtxt(lib_path+'/bearing2_OFF.txt')
#bearing1_off = bearing1_off *180/pi
#bearing2_off = bearing2_off *180/pi
#bearing1_on = bearing1_on *180/pi
#bearing2_on = bearing2_on *180/pi