import matplotlib.pyplot as plt
import numpy as np
import os, importlib
from math import pi
from matplotlib.lines import Line2D
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
target_init = config.TARGET_INIT

#LOAD LOG FILES
# Estimation vs Real (in time)

est4_x_ON = np.loadtxt(lib_path+'/est4_x_ON.txt')
est4_y_ON = np.loadtxt(lib_path+'/est4_y_ON.txt')
real_y = np.loadtxt(lib_path+'/target_y_traj.txt')
real_x = np.loadtxt(lib_path+'/target_x_traj.txt')

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

auv3_x_off = np.loadtxt(lib_path+'/auv3_x_OFF.txt')
auv3_y_off = np.loadtxt(lib_path+'/auv3_y_OFF.txt')
auv4_x_off = np.loadtxt(lib_path+'/auv4_x_OFF.txt')
auv4_y_off = np.loadtxt(lib_path+'/auv4_y_OFF.txt')
# Optimization CTRL_CMDS
ctrl_cmds = np.loadtxt(lib_path+'/plot_cmds.txt')
opt_x = np.loadtxt(lib_path+'/t_est_x_opt.txt')
opt_y = np.loadtxt(lib_path+'/t_est_y_opt.txt')
s_opt_x = np.loadtxt(lib_path+'/s_state_x.txt')
s_opt_y = np.loadtxt(lib_path+'/s_state_y.txt')
# Covariance data
cov1 = np.loadtxt(lib_path+'/cov1_ON.txt')
cov2 = np.loadtxt(lib_path+'/cov2_ON.txt')
cov3 = np.loadtxt(lib_path+'/cov3_ON.txt')
cov4 = np.loadtxt(lib_path+'/cov4_ON.txt')
vx = np.loadtxt(lib_path+'/vx_ON.txt')
vy = np.loadtxt(lib_path+'/vy_ON.txt')
cov1 = np.loadtxt(lib_path+'/cov1_OFF.txt')
cov2 = np.loadtxt(lib_path+'/cov2_OFF.txt')
cov3 = np.loadtxt(lib_path+'/cov3_OFF.txt')
cov4 = np.loadtxt(lib_path+'/cov4_OFF.txt')
vx = np.loadtxt(lib_path+'/vx_OFF.txt')
vy = np.loadtxt(lib_path+'/vy_OFF.txt')
# Load temporal vaiables
n_sample = np.size(est4_x_ON)
t = np.linspace(0,config.TIME_DURATION,n_sample)
scaling = 3 #scale the width of the drawn lines (i.e. 3 suitable for 10km X 10km area)
ranges = 12 #scale the number of printed AUVs (ie.e 12 suitable for 600 s of simulation)

plt.title('TOPOLOGY of the NETWORK with IN-LINE FORMATION',fontsize=30)
plt.plot(auv1_x_off[0],auv1_y_off[0])
circle4 = plt.Circle((auv1_x_off[0],auv1_y_off[0]),1*scaling,color='k')
plt.text(auv1_x_off[0],auv1_y_off[0],'     AUV2',fontsize=20)
circle5 = plt.Circle((auv2_x_off[0],auv2_y_off[0]),1*scaling,color='k')
plt.text(auv2_x_off[0],auv2_y_off[0],'     AUV3',fontsize=20)
circle6 = plt.Circle((auv3_x_off[0],auv3_y_off[0]),1*scaling,color='k')
plt.text(auv3_x_off[0],auv3_y_off[0],'     AUV1',fontsize=20)
circle7 = plt.Circle((auv4_x_off[0],auv4_y_off[0]),1*scaling,color='k')
plt.text(auv4_x_off[0],auv4_y_off[0],'     AUV4',fontsize=20)
plt.gca().add_patch(circle7)
plt.gca().add_patch(circle4)
plt.gca().add_patch(circle5)
plt.gca().add_patch(circle6)
plt.xlabel('x (m)',fontsize=30)
plt.ylabel('y (m)',fontsize=30)
plt.yticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.grid()
plt.axis('equal')
plt.show()

# PLOT THE RESULT OF THE SIMULATION without OPTIMIZATION
plt.plot(s_x_off,s_y_off)
plt.plot(real_x,real_y,linewidth=5,color='y')
plt.plot(est4_x_OFF,est4_y_OFF,'r',linewidth=3)
plt.xlabel('x (m)',fontsize=30)
plt.ylabel('y (m)',fontsize=30)

legend_elements = [Line2D([0], [0], marker='X',color='b', lw=1, label='Formation Reference Path'),
                    Line2D([0], [0], color='yellow', lw=5, label='Target Real Path'),
                    Line2D([0], [0], color='r', lw=1, label='Target Estimation'),
                    Line2D([0], [0], marker='o',color='k',  label='AUVs'),
                    Line2D([0], [0], color='k', ls='--', label='LOS AUVs')]

plt.legend(handles=legend_elements,fontsize=20)
plt.plot(auv1_x_off[10:-1], auv1_y_off[10:-1], 'k')
plt.plot(auv2_x_off[10:-1], auv2_y_off[10:-1], 'k')
plt.plot(auv3_x_off[10:-1], auv3_y_off[10:-1], 'k')
plt.plot(auv4_x_off[10:-1], auv4_y_off[10:-1], 'k')

j = 0
for i in range(ranges):
    # plot LOS
    idx = (i+1)*5*ranges
    if i == ranges:
        idx = -1
    plt.plot([auv3_x_off[idx],
        real_x[idx]],[auv3_y_off[idx],real_y[idx]],'k--',linewidth=1)
    plt.plot([auv4_x_off[idx],real_x[idx]],[auv4_y_off[idx],
        real_y[idx]],'k--',linewidth=1)
    # plot AUVs
    plt.plot(auv1_x_off[idx],auv1_y_off[idx],'ok',linewidth=20)
    plt.plot(auv2_x_off[idx],auv2_y_off[idx],'ok',linewidth=20)
    plt.plot(auv3_x_off[idx],auv3_y_off[idx],'ok',linewidth=20)
    plt.plot(auv4_x_off[idx],auv4_y_off[idx],'ok',linewidth=20)
    circle = plt.Circle((real_x[idx],real_y[idx]),10,color='y')
    plt.gca().add_patch(circle)
    plt.text(real_x[idx],real_y[idx],'t'+str(j+1),fontsize=20)
    # plot REFERENCE
    plt.plot(s_x_off[idx],s_y_off[idx],'Xb',linewidth=1)  
    j += 1 
scaling = 30
plt.arrow(real_x[np.round(0)],real_y[np.round(0)],+5.0*scaling*np.cos(target_init[2]), 5.0*scaling*np.sin(target_init[2]),width=2*scaling,color='y')
plt.axis('equal')
plt.grid()
plt.yticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.show()

# PLOT THE RESULT OF THE SIMULATION with OPTIMIZATION

plt.plot(s_x_on,s_y_on)
plt.plot(real_x,real_y,linewidth=5,color='y')
plt.plot(est4_x_ON,est4_y_ON,'r',linewidth=3)
#plt.title('SIMULATION - OPTIMIZATION ON',fontsize=30)
plt.xlabel('x (m)',fontsize=30)
plt.ylabel('y (m)',fontsize=30)

legend_elements = [Line2D([0], [0], marker='X',color='b', lw=1, label='Formation Reference Path'),
                    Line2D([0], [0], color='yellow', lw=5, label='Target Real Path'),
                    Line2D([0], [0], color='r', lw=1, label='Target Estimation'),
                    Line2D([0], [0], marker='o',color='k',  label='AUVs'),

                    Line2D([0], [0], color='k', ls='--', label='LOS AUVs')]

plt.legend(handles=legend_elements, fontsize=20)

plt.plot(auv1_x_on[10:-1], auv1_y_on[10:-1], 'k')
plt.plot(auv2_x_on[10:-1], auv2_y_on[10:-1], 'k')
plt.plot(auv3_x_on[10:-1], auv3_y_on[10:-1], 'k')
plt.plot(auv4_x_on[10:-1], auv4_y_on[10:-1], 'k')
j = 0
lista = []
for i in range(ranges):

    # plot LOS
    idx = (i+1)*5*ranges
    if i == ranges:
        idx = -1
    plt.plot([auv3_x_on[idx],
        real_x[idx]],[auv3_y_on[idx],real_y[idx]],'k--',linewidth=1)
    plt.plot([auv4_x_on[idx],real_x[idx]],[auv4_y_on[idx],
        real_y[idx]],'k--',linewidth=1)
    # plot AUVs
    plt.plot(auv1_x_on[idx],auv1_y_on[idx],'ok',linewidth=20)
    plt.plot(auv2_x_on[idx],auv2_y_on[idx],'ok',linewidth=20)
    plt.plot(auv3_x_on[idx],auv3_y_on[idx],'ok',linewidth=20)
    plt.plot(auv4_x_on[idx],auv4_y_on[idx],'ok',linewidth=20)
    plt.plot(real_x[idx],real_y[idx],'oy',linewidth=5)#500
    circle = plt.Circle((real_x[idx],real_y[idx]),10,color='y')
    plt.gca().add_patch(circle)
    plt.text(real_x[idx],real_y[idx],'t'+str(j+1),fontsize=20)
    # plot REFERENCE
    plt.plot(s_x_on[idx],s_y_on[idx],'Xb',linewidth=5) 
    #tmp = np.sqrt((auv1_x_on[idx]-auv4_x_on[idx])**2+(auv1_y_on[idx]-auv4_y_on[idx])**2)
    #circle = plt.Circle((s_x_on[idx],s_y_on[idx]),radius=tmp/2,fill=False)
    #plt.scatter(s_x_on[idx],s_y_on[idx], s=tmp, facecolors='none', edgecolors='r')
    plt.gca().add_patch(circle)
    j += 1 
plt.axis('equal')
idx1 = int(len(real_x)/2) 
plt.arrow(real_x[np.round(0)],real_y[np.round(0)],+5.0*scaling*np.cos(target_init[2]), 5.0*scaling*np.sin(target_init[2]),width=2*scaling,color='y')
plt.grid()
plt.yticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.show()

# PLOT COVARIANCE OBERVATIONS

plt.subplot(2,1,1)
plt.plot(t,cov1)
plt.plot(t,cov2)
plt.legend(['x','y'])
plt.grid()
plt.subplot(2,1,2)
plt.plot(t,cov3)
plt.plot(t,cov4)
plt.legend(['vx','vy'])
plt.grid()
plt.show()

real_vel = config.TARGET_VEL
vx_real = real_vel*np.cos(config.TARGET_INIT[2])
vy_real = real_vel*np.sin(config.TARGET_INIT[2])
y1 = []
y2 = []
for i in range(len(t)):
    y1.append(vx_real)
    y2.append(vy_real)
plt.plot(t,vx)
plt.plot(t,vy)
plt.plot(t,y1,'r--')
plt.plot(t,y2,'r:')
plt.legend(['vx','vy','real_x','real_y'])
plt.grid()
plt.show()


# PLTO BASELINE ANGLE
baseline_angle = []
for i in range(len(auv4_x_on)-8):
    tmp = (((real_x[i]-auv3_x_on[i])*(real_x[i]-auv4_x_on[i]))+((real_y[i]-auv3_y_on[i])*(real_y[i]-auv4_y_on[i]))
    )/(np.sqrt((real_x[i]-auv3_x_on[i])**2+(real_y[i]-auv3_y_on[i])**2)*np.sqrt((real_x[i]-auv4_x_on[i])**2+(
        real_y[i]-auv4_y_on[i])**2))

    angle = np.arccos(tmp)
    angle = angle*180/pi
    baseline_angle.append(angle)

baseline_angle_off = []
for i in range(len(auv4_x_off)-8):
    tmp = (((real_x[i]-auv3_x_off[i])*(real_x[i]-auv4_x_off[i]))+((real_y[i]-auv3_y_off[i])*(real_y[i]-auv4_y_off[i]))
    )/(np.sqrt((real_x[i]-auv3_x_off[i])**2+(real_y[i]-auv3_y_off[i])**2)*np.sqrt((real_x[i]-auv4_x_off[i])**2+(
        real_y[i]-auv4_y_off[i])**2))

    angle = np.arccos(tmp)
    angle = angle*180/pi
    baseline_angle_off.append(angle)

y = np.zeros(n_sample)
plt.subplot(3,1,1)
plt.title('OPTIMIZATION ON',fontsize=15)
plt.plot(t,baseline_angle,'k',markerfacecolor='yellow')
plt.ylabel('Angle LOS1 & LOS4 (deg)',fontsize=20)
plt.yticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.grid()

plt.subplot(3,1,2)
plt.title('OPTIMIZATION OFF',fontsize=15)
plt.plot(t,baseline_angle_off,'k',markerfacecolor='yellow')
plt.yticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.grid()

# PLOT RMSE
sum1 = 0
sum2 = 0
for i in range(len(err_off)):
    tmp1 = err_off[i]
    sum1 += tmp1
    tmp2 = err_on[i]
    sum2 += tmp2
err_medio1 = np.sqrt(sum1/n_sample)
err_medio2 = np.sqrt(sum2/n_sample)
print('ERRORE MEDIO OFF:',err_medio1/10)
print('ERRORE MEDIO ON:',err_medio2/10)

# COMPARE RMSE 
plt.subplot(3,1,3)
plt.plot(t,err_off[0:n_sample]/10,'b')
plt.plot(t,err_on[0:n_sample]/10,'g')
plt.legend(['optimization OFF','optimization ON'],fontsize=20)

plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('RMSE (m)',fontsize=30)
plt.yticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.grid()
plt.show()

# PLOT ctrl cmds from optimization
n_sample1 = np.size(ctrl_cmds)
t1 = np.linspace(640,config.TIME_DURATION,n_sample1)
plt.plot(t1,ctrl_cmds*180/pi,'-ok',markerfacecolor='blue')
plt.title('HEADING CHANGE COMMANDED',fontsize=20)
plt.xlabel('Time (s)',fontsize=20)
plt.ylabel('Heading Changes (deg)',fontsize=20)
plt.grid()
plt.show()

plt.plot(s_opt_x,s_opt_y)
plt.plot(opt_x,opt_y)
plt.plot(real_x,real_y)
plt.grid()
plt.axis('equal')
plt.show()