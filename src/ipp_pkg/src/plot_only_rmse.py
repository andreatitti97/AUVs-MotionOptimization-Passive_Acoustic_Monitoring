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
scaling = 6 #scale the width of the drawn lines (i.e. 3 suitable for 10km X 10km area)
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
plt.plot(t,((err_on[0:n])),'g',marker='o',markerfacecolor='g')
y_on = []
y_off = []
for i in range(len(t)):
    y_off.append(errore_medio_off)
    y_on.append(errore_medio_on)
#plt.plot(t,(y_off),'b--')
#plt.plot(t,(y_on),'g--')
plt.legend(['optimization OFF','optimization ON'],fontsize=20)

plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('Residual Error (m)',fontsize=30)
plt.yticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.grid()
plt.show()