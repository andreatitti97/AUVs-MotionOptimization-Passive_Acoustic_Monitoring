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

err_on = np.loadtxt(lib_path+'/err_quad_ON.txt')
err_off = np.loadtxt(lib_path+'/err_quad_OFF.txt')
err_fusion = np.loadtxt(lib_path+'/err_fusion_OFF.txt')

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
ctrl_cmds = np.loadtxt(lib_path+'/plot_cmds_off.txt')
ctrl_cmds_off = np.loadtxt(lib_path+'/plot_cmds_off.txt')
opt_x = np.loadtxt(lib_path+'/t_est_x_opt.txt')
opt_y = np.loadtxt(lib_path+'/t_est_y_opt.txt')
s_opt_x = np.loadtxt(lib_path+'/s_state_x.txt')
s_opt_y = np.loadtxt(lib_path+'/s_state_y.txt')
# Covariance data
cov1_ON = np.loadtxt(lib_path+'/cov1_ON.txt')
cov2_ON = np.loadtxt(lib_path+'/cov2_ON.txt')
cov3_ON = np.loadtxt(lib_path+'/cov3_ON.txt')
cov4_ON = np.loadtxt(lib_path+'/cov4_ON.txt')
vx_ON = np.loadtxt(lib_path+'/vx_ON.txt')
vy_ON = np.loadtxt(lib_path+'/vy_ON.txt')
cov1_OFF = np.loadtxt(lib_path+'/cov1_OFF.txt')
cov2_OFF = np.loadtxt(lib_path+'/cov2_OFF.txt')
cov3_OFF = np.loadtxt(lib_path+'/cov3_OFF.txt')
cov4_OFF = np.loadtxt(lib_path+'/cov4_OFF.txt')
vx_OFF = np.loadtxt(lib_path+'/vx_OFF.txt')
vy_OFF = np.loadtxt(lib_path+'/vy_OFF.txt')

# Plot Parameters
scaling = 10 #scale the width of the drawn lines (i.e. 3 suitable for 10km X 10km area)
ranges = 12 #scale the number of printed AUVs (ie.e 12 suitable for 600 s of simulation)

# Plot Tracking Error and compute RMSE
sum1, sum2, sum3 = 0,0,0

errore_medio_off = sum(err_off)/len(err_off)
errore_medio_on = sum(err_on)/len(err_on)
print('errore medio OFF',errore_medio_off)
print('errore medio ON',errore_medio_on)
print('MSE OFF',sum(err_off**2)/len(err_off))
print('MSE ON',sum(err_on**2)/len(err_on))
rmse_off = np.sqrt(sum(err_off**2)/len(err_off))
rmse_on = np.sqrt(sum(err_on**2)/len(err_on))
print('RMSE OFF:',np.round(rmse_off,3))
print('RMSE ON:',np.round(rmse_on,3))

sum1,sum2,sum3 = 0,0,0

for i in range(len(err_off)):
    sum1 += ((err_off[i])-errore_medio_off)**2
for i in range(len(err_on)):
    sum2 += ((err_on[i])-errore_medio_on)**2

dev_std_off = np.sqrt(sum1/len(err_off))
dev_std_on = np.sqrt(sum2/len(err_on))

print('VARIANZA OFF',np.float32(dev_std_off**2))
print('VARIANZA ON',np.float32(dev_std_on**2))
print('DEV STD OFF',np.float32(np.round(dev_std_off,3)))
print('DEV STD ON',np.round(dev_std_on,3))

# Magnitude Tracking Error plot
if len(err_off) < len(err_on):
    n = len(err_off)
else:
    n = len(err_on)
t = np.linspace(0,config.TIME_DURATION,n)
plt.plot(t,(err_off[0:n]),'b',marker='o',markerfacecolor='b') #err_off contiene e(t) = ex + ey, dove ex = (x - x_hat)**2
plt.plot(t,(err_on[0:n]),'g',marker='o',markerfacecolor='g')
y_on = []
y_off = []
for i in range(len(t)):
    y_off.append(errore_medio_off)
    y_on.append(errore_medio_on)
plt.plot(t,(y_off),'b--')
plt.plot(t,(y_on),'g--')
plt.legend(['optimization OFF','optimization ON'],fontsize=20)

plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('Residual Error (m)',fontsize=30)
plt.yticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.grid() 
plt.show()


# Plot Topology of the network
plt.title('TOPOLOGY of the NETWORK',fontsize=30)
plt.plot(auv1_x_off[0],auv1_y_off[0])
circle4 = plt.Circle((auv1_x_off[0],auv1_y_off[0]),1*scaling,color='k')
plt.text(auv1_x_off[0],auv1_y_off[0],'     AUV1',fontsize=20)
circle5 = plt.Circle((auv2_x_off[0],auv2_y_off[0]),1*scaling,color='k')
plt.text(auv2_x_off[0],auv2_y_off[0],'     AUV2',fontsize=20)
circle6 = plt.Circle((auv3_x_off[0],auv3_y_off[0]),1*scaling,color='k')
plt.text(auv3_x_off[0],auv3_y_off[0],'     AUV3',fontsize=20)
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


# PLOT THE OUTPUT OF THE SIMULATOR without OPTIMIZATION
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

plt.plot(s_x_off[0],s_y_off[0],'ob')
plt.plot(auv1_x_off[0],auv1_y_off[0],'og',linewidth=20)
plt.plot(auv2_x_off[0],auv2_y_off[0],'ok',linewidth=20)
plt.plot(auv3_x_off[0],auv3_y_off[0],'ok',linewidth=20)
plt.plot(auv4_x_off[0],auv4_y_off[0],'og',linewidth=30)
j = 0
for i in range(ranges):
    # plot LOS
    
    idx = (i+1)*5*ranges
    
    plt.plot([auv1_x_off[idx],
        real_x[idx]],[auv1_y_off[idx],real_y[idx]],'k--',linewidth=1)
    plt.plot([auv4_x_off[idx],real_x[idx]],[auv4_y_off[idx],
        real_y[idx]],'k--',linewidth=1)

    # plot AUVs
    plt.plot(auv1_x_off[idx],auv1_y_off[idx],'ok',linewidth=20)
    plt.plot(auv2_x_off[idx],auv2_y_off[idx],'ok',linewidth=20)
    plt.plot(auv3_x_off[idx],auv3_y_off[idx],'ok',linewidth=20)
    plt.plot(auv4_x_off[idx],auv4_y_off[idx],'ok',linewidth=20)
    circle = plt.Circle((real_x[idx],real_y[idx]),10,color='y')
    plt.gca().add_patch(circle)
    # plot TARGET and FORMATION REFERENCE
    plt.text(real_x[idx],real_y[idx],'t'+str(j+1),fontsize=20)
    plt.plot(s_x_off[idx],s_y_off[idx],'Xb',linewidth=1)  
    j += 1

scaling = 10
plt.arrow(real_x[np.round(0)],real_y[np.round(0)],+5.0*scaling*np.cos(target_init[2]), 5.0*scaling*np.sin(target_init[2]),width=2*scaling,color='y')
plt.axis('equal')
plt.grid()
plt.yticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.show()

# PLOT THE RESULT OF THE SIMULATION with OPTIMIZATION

#plt.plot(s_x_on,s_y_on)
plt.plot(real_x,real_y,linewidth=5,color='y')
plt.plot(est4_x_ON,est4_y_ON,'r',linewidth=3)
plt.xlabel('x (m)',fontsize=30)
plt.ylabel('y (m)',fontsize=30)

legend_elements = [Line2D([0], [0], marker='X',color='b', lw=1, label='Formation Reference Path'),
                    Line2D([0], [0], color='yellow', lw=5, label='Target Real Path'),
                    Line2D([0], [0], color='r', lw=1, label='Target Estimation'),
                    Line2D([0], [0], marker='o',color='k',  label='AUVs'),
                    Line2D([0], [0], color='k', ls='--', label='LOS AUVs')]

plt.legend(handles=legend_elements, fontsize=20)

j = 0
lista = []
plt.plot(s_x_on[0],s_y_on[1],'ob')
plt.plot(auv1_x_on[0],auv1_y_on[0],'ok',linewidth=20)
plt.plot(auv2_x_on[0],auv2_y_on[0],'ok',linewidth=20)
plt.plot(auv3_x_on[0],auv3_y_on[0],'ok',linewidth=20)
plt.plot(auv4_x_on[0],auv4_y_on[0],'ok',linewidth=20)

for i in range(ranges):

    # plot LOS
    idx = (i+1)*5*ranges
    if i == ranges:
        idx = -1
    plt.plot([auv1_x_on[idx],
        real_x[idx]],[auv1_y_on[idx],real_y[idx]],'k--',linewidth=1)
    plt.plot([auv4_x_on[idx],real_x[idx]],[auv4_y_on[idx],
        real_y[idx]],'k--',linewidth=1)

    plt.plot([auv1_x_on[idx],
        s_x_on[idx]],[auv1_y_on[idx],s_y_on[idx]],'g--',linewidth=1)

    # plot AUVs
    plt.plot(auv1_x_on[idx],auv1_y_on[idx],'ok',linewidth=20)
    plt.plot(auv2_x_on[idx],auv2_y_on[idx],'ok',linewidth=20)
    plt.plot(auv3_x_on[idx],auv3_y_on[idx],'ok',linewidth=20)
    plt.plot(auv4_x_on[idx],auv4_y_on[idx],'ok',linewidth=20)
    plt.plot(real_x[idx],real_y[idx],'oy',linewidth=5)#500
    circle = plt.Circle((real_x[idx],real_y[idx]),10,color='y')
    plt.gca().add_patch(circle)
    # plot target and formation reference
    plt.text(real_x[idx],real_y[idx],'t'+str(j+1),fontsize=20)
    plt.plot(s_x_on[idx],s_y_on[idx],'Xb',linewidth=5) 
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

if len(t) >= len(cov1_OFF):
    n = len(cov1_OFF)
elif len(t) < len(cov1_OFF):
    n = len(t)

plt.subplot(4,1,1)
plt.plot(t[0:n],cov1_ON[0:n])
plt.plot(t[0:n],cov1_OFF[0:n])
plt.legend(['x_ON','x_OFF'])
plt.grid()
plt.subplot(4,1,2)
plt.plot(t[0:n],cov2_ON[0:n])
plt.plot(t[0:n],cov2_OFF[0:n])
plt.legend(['y_ON','y_OFF'])
plt.grid()
plt.subplot(4,1,3)
plt.plot(t[0:n],cov3_ON[0:n])
plt.plot(t[0:n],cov3_OFF[0:n])
plt.grid()
plt.legend(['vx_ON','vx_OFF'])
plt.subplot(4,1,4)
plt.plot(t[0:n],cov4_ON[0:n])
plt.plot(t[0:n],cov4_OFF[0:n])
plt.legend(['vy_ON','vy_ON'])
plt.grid()
plt.show()

# Compute TRACKING ANGLE
if len(t) >= len(auv4_x_on):
    n = len(auv4_x_on)
elif len(t) < len(auv4_x_on):
    n = len(t)

t_off = np.linspace(0,config.TIME_DURATION,len(auv4_x_off))
t_on = np.linspace(0,config.TIME_DURATION,len(auv4_x_on))
baseline_angle = []

for i in range(len(auv4_x_on)):
    tmp = (((real_x[i]-auv1_x_on[i])*(real_x[i]-auv4_x_on[i]))+((real_y[i]-auv1_y_on[i])*(real_y[i]-auv4_y_on[i]))
    )/(np.sqrt((real_x[i]-auv1_x_on[i])**2+(real_y[i]-auv1_y_on[i])**2)*np.sqrt((real_x[i]-auv4_x_on[i])**2+(
        real_y[i]-auv4_y_on[i])**2))

    angle = np.arccos(tmp)
    angle = angle*180/pi
    baseline_angle.append(angle)

baseline_angle_off = []
for i in range(len(auv4_x_off)):
    tmp = (((real_x[i]-auv1_x_off[i])*(real_x[i]-auv4_x_off[i]))+((real_y[i]-auv1_y_off[i])*(real_y[i]-auv4_y_off[i]))
    )/(np.sqrt((real_x[i]-auv1_x_off[i])**2+(real_y[i]-auv1_y_off[i])**2)*np.sqrt((real_x[i]-auv4_x_off[i])**2+(
        real_y[i]-auv4_y_off[i])**2))

    angle = np.arccos(tmp)
    angle = angle*180/pi
    baseline_angle_off.append(angle)

# Plot TRACKING ANGLE and TRACKING ERROR
y = np.zeros(n)
plt.subplot(2,1,1)
plt.title('PERFORMANCES COMPARISON',fontsize=30)
plt.plot(t_off,baseline_angle,'g',markerfacecolor='yellow')
plt.plot(t_off,baseline_angle_off,'b',markerfacecolor='yellow')
plt.ylabel('Angle LOS (deg)',fontsize=22)
plt.yticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.legend(['OPTIMIZATION ON','OPTIMIZATION OFF'],fontsize=22)
plt.grid()

plt.subplot(2,1,2)
plt.plot(t[0:n],(err_off[0:n]),'b',marker='o',markerfacecolor='b') #err_off contiene e(t) = ex + ey, dove ex = (x - x_hat)**2
plt.legend(['ADAPTATION OFF'])
plt.plot(t[0:n],(err_on[0:n]),'g',marker='o',markerfacecolor='g')

y_on = []
y_off = []
for i in range(len(t)):
    y_off.append(errore_medio_off)
    y_on.append(errore_medio_on)
plt.plot(t,y_off,'b--')
plt.plot(t,y_on,'g--')
plt.legend(['optimization OFF','optimization ON','RMSE off','RMSE on'],fontsize=15)

plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('Tracking Error (m)',fontsize=30)
plt.yticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.grid()
plt.show()

# PLOT ctrl cmds from optimization
n_sample1 = np.size(ctrl_cmds)
t1 = np.linspace(640,config.TIME_DURATION,n_sample1)
plt.subplot(2,1,1)
plt.title('HEADING CHANGE COMMANDED',fontsize=20)
plt.plot(t1,ctrl_cmds*180/pi,'-ok',markerfacecolor='blue')
plt.yticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.ylabel('Heading Changes (deg)',fontsize=20)
plt.legend(['ADAPTATION ON'],fontsize=20)
plt.grid()

# PLOT ctrl cmds from optimization
'''plt.subplot(2,1,2)
n_sample1 = np.size(ctrl_cmds_off)
t1 = np.linspace(640,config.TIME_DURATION,n_sample1)
plt.plot(t1,ctrl_cmds_off*180/pi,'-ok',markerfacecolor='blue')
plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('Heading Changes (deg)',fontsize=20)
plt.legend(['ADAPTATION OFF'],fontsize=20)
plt.grid()
plt.yticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.show()'''

'''# Output of the prediction phase during optimization:

plt.plot(s_opt_x,s_opt_y)
plt.plot(opt_x,opt_y)
plt.plot(real_x,real_y)
plt.grid()
plt.axis('equal')
plt.show()'''