import matplotlib.pyplot as plt
import numpy as np

import math
import os, importlib
from math import pi
from matplotlib.lines import Line2D

lib_path = os.path.abspath('/home/andrea/Scrivania/test_paper_OFFICIAL/target_dynamics/')
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
target_init = config.TARGET_INIT

dyn1_x = np.loadtxt(lib_path+'/target_x_traj1.txt')
dyn1_y = np.loadtxt(lib_path+'/target_y_traj1.txt')
dyn2_x = np.loadtxt(lib_path+'/target_x_traj2.txt')
dyn2_y = np.loadtxt(lib_path+'/target_y_traj2.txt')
dyn3_x = np.loadtxt(lib_path+'/target_x_traj3.txt')
dyn3_y = np.loadtxt(lib_path+'/target_y_traj3.txt')
dyn4_x = np.loadtxt(lib_path+'/target_x_traj4.txt')
dyn4_y = np.loadtxt(lib_path+'/target_y_traj4.txt')
dyn5_x = np.loadtxt(lib_path+'/target_x_traj5.txt')
dyn5_y = np.loadtxt(lib_path+'/target_y_traj5.txt')
dyn6_x = np.loadtxt(lib_path+'/target_x_traj6.txt')
dyn6_y = np.loadtxt(lib_path+'/target_y_traj6.txt')
dyn7_x = np.loadtxt(lib_path+'/target_x_traj7.txt')
dyn7_y = np.loadtxt(lib_path+'/target_y_traj7.txt')
dyn8_x = np.loadtxt(lib_path+'/target_x_traj8.txt')
dyn8_y = np.loadtxt(lib_path+'/target_y_traj8.txt')

# Plot parameters
scaling = 100.0
lw = 2*4*2
lw_ms = 2*7
fs = 16*2 #14
ms = 12*2 #10
ms_arrow= 200
fig = plt.figure(figsize=(8, 4))
plt.xlabel('x [m]',fontsize=fs)
plt.ylabel('y [m]',fontsize=fs)
plt.title('Motions of the target',weight='bold',fontsize=fs)
legend_elements = [Line2D([0], [0], marker='o',color='y', lw=lw, markersize=ms, label='s(t0)'),
                    Line2D([0], [0], color='tab:blue', lw=lw/2, label='Dynamic 1'),
                    Line2D([0], [0],color='tab:orange',lw=lw/2, label='Dynamic 2'),
                    Line2D([0], [0],color='tab:green',lw=lw/2, label='Dynamic 3'),
                    Line2D([0], [0],color='tab:pink',lw=lw/2, label='Dynamic 4'),
                    Line2D([0], [0],color='tab:red',lw=lw/2, label='Dynamic 5'),
                    Line2D([0], [0],color='tab:brown',lw=lw/2, label='Dynamic 6'),
                    Line2D([0], [0],color='tab:cyan',lw=lw/2, label='Dynamic 7'),
                    Line2D([0], [0],color='tab:purple',lw=lw/2, label='Dynamic 8')]

plt.legend(handles=legend_elements, fontsize=fs)

plt.plot(0,0,'oy',markersize=lw_ms)

plt.plot(dyn2_x,dyn2_y,'tab:orange',linewidth=lw/2)
plt.plot(dyn3_x,dyn3_y,'tab:green',linewidth=lw/2)

plt.plot(dyn5_x,dyn5_y,'tab:red',linewidth=lw/2)
plt.plot(dyn6_x,dyn6_y,'tab:brown',linewidth=lw/2)
plt.plot(dyn7_x,dyn7_y,'tab:cyan',linewidth=lw/2)
plt.plot(dyn8_x,dyn8_y,'tab:purple',linewidth=lw/2)
plt.plot(dyn1_x,dyn1_y,'tab:blue',linewidth=lw/2)
plt.plot(dyn4_x,dyn4_y,'tab:pink',linewidth=lw/2)
#plt.legend(['s(t0)','Dynamic 1','Dynamic 2','Dynamic 3','Dynamic 4','Dynamic 5','Dynamic 6','Dynamic 7','Dynamic 8'])
plt.arrow(dyn1_x[-1],dyn1_y[-1],np.cos(pi), np.sin(pi),width=ms_arrow,color='tab:blue')
plt.arrow(dyn2_x[-1],dyn2_y[-1],np.cos(140*pi/180), np.sin(140*pi/180),width=ms_arrow,color='tab:orange')
plt.arrow(dyn3_x[-1],dyn3_y[-1],np.cos(pi/4), np.sin(pi/4),width=ms_arrow,color='tab:green')
plt.arrow(dyn4_x[-1],dyn4_y[-1],np.cos(-pi/10), np.sin(-pi/10),width=ms_arrow,color='tab:pink')
plt.arrow(dyn5_x[-1],dyn5_y[-1],np.cos(pi/2), np.sin(pi/2),width=ms_arrow,color='tab:red')
plt.arrow(dyn6_x[-1],dyn6_y[-1],np.cos(3*pi/2), np.sin(3*pi/2),width=ms_arrow,color='tab:brown')
plt.arrow(dyn7_x[-1],dyn7_y[-1],np.cos(pi/6), np.sin(pi/6),width=ms_arrow,color='tab:cyan')
plt.arrow(dyn8_x[-1],dyn8_y[-1],np.cos(pi/8), np.sin(pi/8),width=ms_arrow,color='tab:purple')
plt.axis('equal')
plt.grid(linewidth=0.5)
plt.yticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.show()
