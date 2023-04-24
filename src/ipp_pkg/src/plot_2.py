import matplotlib.pyplot as plt
import numpy as np
import os, importlib
from math import pi
from matplotlib.lines import Line2D
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')
M_1_path = os.path.abspath('/home/andrea/Scrivania/TEST_PAPER/preliminary/test_al_variare_M/M1')
M_2_path = os.path.abspath('/home/andrea/Scrivania/TEST_PAPER/preliminary/test_al_variare_M/M2')
M_3_path = os.path.abspath('/home/andrea/Scrivania/TEST_PAPER/preliminary/test_al_variare_M/M3')
M_4_path = os.path.abspath('/home/andrea/Scrivania/TEST_PAPER/preliminary/test_al_variare_M/M4')
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

prova_x = np.loadtxt(lib_path+'/prova_x.txt')
prova_y = np.loadtxt(lib_path+'/prova_y.txt')


# RMSE al variare di M
err_M_1 = np.loadtxt(M_1_path+'/rmse_ON.txt')
err_M_2 = np.loadtxt(M_2_path+'/rmse_ON.txt')
err_M_3 = np.loadtxt(M_3_path+'/rmse_ON.txt')
err_M_4 = np.loadtxt(M_4_path+'/rmse_ON.txt')
err_off = np.loadtxt(lib_path+'/rmse_OFF.txt')
# Path produced at M changing
x_off = np.loadtxt(lib_path+'/x_platform_OFF.txt')
y_off = np.loadtxt(lib_path+'/y_platform_OFF.txt')
x_1 = np.loadtxt(M_1_path+'/x_platform_ON.txt')
y_1 = np.loadtxt(M_1_path+'/y_platform_ON.txt')
x_2 = np.loadtxt(M_2_path+'/x_platform_ON.txt')
y_2 = np.loadtxt(M_2_path+'/y_platform_ON.txt')
x_3 = np.loadtxt(M_3_path+'/x_platform_ON.txt')
y_3 = np.loadtxt(M_3_path+'/y_platform_ON.txt')
x_4 = np.loadtxt(M_4_path+'/x_platform_ON.txt')
y_4 = np.loadtxt(M_4_path+'/y_platform_ON.txt')

plt.plot(x_off,y_off)
plt.plot(x_1,y_1,markerfacecolor='r')
plt.plot(x_2,y_2)
plt.plot(x_3,y_3)
plt.plot(x_4,y_4)
plt.legend(['off','M=1','M=2','M=3','M=4'])
plt.grid()
plt.show()

# Load temporal vaiables
n_sample = np.size(est4_x_ON)
t = np.linspace(0,config.TIME_DURATION,n_sample)
scaling = 6 #scale the width of the drawn lines (i.e. 3 suitable for 10km X 10km area)
ranges = 12 #scale the number of printed AUVs (ie.e 12 suitable for 600 s of simulation)

# PLOT RMSE
sum0 = 0
sum1 = 0
sum2 = 0
sum3 = 0
sum4 = 0

n = 0
if len(err_off) < len(err_M_1):
    n = len(err_off)
else:
    n = len(err_M_1)

if n > len(err_M_2):
    n = len(err_M_2)
else: 
    n = n

if n > len(err_M_3):
    n = len(err_M_3)
else: 
    n = n

if n > len(err_M_4):
    n = len(err_M_4)
else: 
    n = n


for i in range(n):
    tmp0 = err_off[i]
    sum0 += tmp0
    tmp1 = err_M_1[i]
    sum1 += tmp1
    tmp2 = err_M_2[i]
    sum2 += tmp2
    tmp3 = err_M_3[i]
    sum3 += tmp3
    tmp4 = err_M_4[i]
    sum4 += tmp3
err_medio0 = np.sqrt(sum0/n_sample)
err_medio1 = np.sqrt(sum1/n_sample)
err_medio2 = np.sqrt(sum2/n_sample)
err_medio3 = np.sqrt(sum3/n_sample)
err_medio4 = np.sqrt(sum4/n_sample)

print('ERRORE MEDIO OFF:',err_medio0)
print('ERRORE MEDIO M=1:',err_medio1)
print('ERRORE MEDIO M=2:',err_medio2)
print('ERRORE MEDIO M=3:',err_medio3)
print('ERRORE MEDIO M=4:',err_medio4)

n_sample = n
t = np.linspace(0,config.TIME_DURATION,n_sample)
# COMPARE RMSE 
#plt.subplot(3,1,3)
plt.plot(t,err_off[0:n_sample],'b',marker='o',markerfacecolor='b')
plt.plot(t,err_M_1[0:n_sample],'g',marker='o',markerfacecolor='g')
plt.plot(t,err_M_2[0:n_sample],'m',marker='o',markerfacecolor='m')
plt.plot(t,err_M_3[0:n_sample],'k',marker='o',markerfacecolor='k')
plt.plot(t,err_M_4[0:n_sample],'r',marker='o',markerfacecolor='r')
plt.legend(['optimization OFF','M=1','M=2','M=3','M=4'],fontsize=20)

plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('RMSE (m)',fontsize=30)
plt.yticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=15, rotation=0)#to set dimension and orientation of tick labels
plt.grid()
plt.show()

