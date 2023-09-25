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

# Plot parameters
scaling = 100.0

plt.plot(dyn1_x,dyn1_y)
plt.plot(dyn2_x,dyn2_y)


plt.legend(['Dynamic 1','Dynamic 2'])
plt.arrow(dyn1_x[-1],dyn1_y[-1],-5.0*scaling*np.cos(math.atan2(dyn1_y[-1],dyn1_x[-1])), -5.0*scaling*np.sin(math.atan2(dyn1_y[-1],dyn1_x[-1])),width=2*scaling,color='y')
plt.axis('equal')
plt.grid()
plt.yticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.xticks(fontsize=25, rotation=0)#to set dimension and orientation of tick labels
plt.show()
