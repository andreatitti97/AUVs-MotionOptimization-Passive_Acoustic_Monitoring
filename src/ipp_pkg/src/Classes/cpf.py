import numpy as np
import matplotlib.pyplot as plt
import time, os
from math import atan2, pi
import importlib.util
# Import Costum classes
class_path = os.path.abspath('/home/andrea/Desktop/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)

spec = importlib.util.spec_from_file_location("module.utils", class_path+"/utils.py")
utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(utils)

spec = importlib.util.spec_from_file_location("module.planner", class_path+"/spline_planner.py")
planner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(planner)
cubicSpline = planner

class CooperativePathFollowing:

    def __init__(self, n_agents, k_att, k_rep, d_rep, bool):
        # Input attribute loading
        self.n_agents = n_agents
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_rep = d_rep
        # Loading attributes from config file
        self.geometry = config.geometry
        self.ko = config.GAIN_YAW_RATE
        self.v_max = config.AUV_MAX_VEL
        self.v_n = config.AUV_VEL
        self.ds = config.TIME_STEP*config.TIME_SCALER #curve sampling, if higher path less smooth (i think)
        self.DT = config.OPTIMIZATION_TIME_STEP
        self.dt = int(self.DT/4)
        self.spline_course = utils.calc_spline_course
        # Initialize waypoints and boolean
        self.ax = []
        self.ay = []
        self.bool = bool


    def initialize_path(self, f):#f=formation
        if self.geometry == 'column' or self.geometry == 'column2':

            d = f[0]-f[1]
            ax_0 =[*range(0,  f[0]+2*self.DT, self.dt)]
            tmp = [*range(0, f[0]+self.DT,  self.dt)]

            ay_0 = []
            for i in range(len(tmp)):
                ay_0.append(0)
            leader_path = cubicSpline.CubicSpline2D(tmp, ay_0)            
            [rx, ry, ryaw, rk, s]= self.spline_course(leader_path,self.ds)
            
            path_idx = len(ryaw)
            d = leader_path.s[-1]
            for i in range(len(ax_0)):
                self.ax.append(ax_0[i])
                self.ay.append(0)
            path = cubicSpline.CubicSpline2D(self.ax, self.ay)

        elif self.geometry == 'line' or self.geometry == 'line2':
            
            ax_0 =[*range(-self.dt,  self.DT,self.dt)]
            d = self.dt

            for i in range(len(ax_0)):
                self.ax.append(ax_0[i])
                self.ay.append(0)
            path = cubicSpline.CubicSpline2D(self.ax, self.ay)
            [rx, ry, ryaw, rk, s]= self.spline_course(path,self.ds)
            int_list = [int(item) for item in rx]

            path_idx = int_list.index(int(d),0,len(int_list))
        

        return path, path_idx, d, self.ax, self.ay

    def saturateVel(self,vel,bool=False):
        if bool == False:
            if vel > self.v_max:
                print('SATURATED VELS +++++++++++++++++++++++++++++++++++++++++++++++++ ',vel)
                vel = self.v_max
            if vel < -self.v_max:
                print('SATURATED VELS ------------------------------------------------- ',vel)
                vel = -self.v_max         
        return vel

    def potential_field(self, path, s_pose, auvs_xy, d):
        # Calculate the desired auvs_xy of the followers in the f
        f = config.formation #THIS IS MANDATORY - DO NOT EDIT
        des_xy = np.zeros_like(auvs_xy)
 
        # Compute the desired absolute auvs_xy of the agents according to leader auvs_xy and given f
        if self.geometry == 'line' or self.geometry == 'line2':
            for i in range(0, self.n_agents):
                des_xy[i,0] = s_pose[0] + (f[i,0]*np.cos(s_pose[2])+f[i,1]*np.sin(s_pose[2]))
                des_xy[i,1] = s_pose[1] - (-f[i,0]*np.sin(s_pose[2])+f[i,1]*np.cos(s_pose[2]))
        elif self.geometry == 'column' or self.geometry == 'column2':
            for i in range(len(f)):                
                x,y = path.calc_position(-f[i]+d)
                des_xy[i,0] = x
                des_xy[i,1] = y

        # Calculate the attractive potential for each robot
        F_att = -self.k_att * (auvs_xy - des_xy)

        # Calculate the repulsive potential for each robot
        F_rep = np.zeros_like(F_att)
        for i in range(self.n_agents):
            for j in range(i+1, self.n_agents):
                dist = np.linalg.norm(auvs_xy[i] - auvs_xy[j])
                if dist < self.d_rep:
                    F_rep[i] += self.k_rep * (1/(dist+1e8) - 1/self.d_rep) * (auvs_xy[i] - auvs_xy[j]) / (dist+1e8)
                    F_rep[j] += self.k_rep * (1/(dist+1e8) - 1/self.d_rep) * (auvs_xy[j] - auvs_xy[i]) / (dist+1e8)
        
        # Calculate the total force for each robot
        F_total = F_att + F_rep

        return F_total, des_xy

    def compute_orientations(self, desired_pos, auvs_xy, auvs_theta):
        orientations_goal = np.zeros((self.n_agents))
        error_ang = np.zeros((self.n_agents))
        for i in range(self.n_agents):
            # Compute the direction vector from the follower's current position to its desired position
            orientations_goal[i] = atan2(auvs_xy[i,1]-desired_pos[i,1],auvs_xy[i,0]-desired_pos[i,0])
            error_ang[i] = (orientations_goal[i] - auvs_theta[i])

        return error_ang

    def update_path(self, waypoints, t_i, DT, ax, ay,d,s_pose):
        self.ax = ax
        self.ay = ay
        tmp_x = ax[-1]
        tmp_y = ay[-1]
        tmp0_x = np.abs(s_pose[0] - self.ax[0])
        tmp0_y = np.abs(s_pose[1] - self.ay[0])
        for i in range(len(ax)):
            tmp2_x = np.abs(s_pose[0] - self.ax[i])
            tmp2_y = np.abs(s_pose[1] - self.ay[i])
            if tmp2_x >= s_pose[0] and tmp2_y >= s_pose[1]:
                if tmp2_x<=tmp0_x:
                    tmp_x = self.ax[i]
                    tmp0_x = tmp2_x
                    if tmp2_y<tmp0_y:
                        tmp_y = self.ay[i]
                        tmp0_y = tmp2_y
            

        idx_x = self.ax.index(int(np.floor(tmp_x)),0,len(self.ax))
        idx_y = self.ay.index(int(np.floor(tmp_y)),0,len(self.ay))
        if idx_x >= idx_y:
            idx = idx_x
        else:
            idx = idx_y

        
        for i in range(len(self.ax[idx:-1])):

            self.ax.pop(-1)
            self.ay.pop(-1)

        path = cubicSpline.CubicSpline2D(self.ax, self.ay)
       
        d = path.s[-1]

        # Compute the distance travelled according to the new path
        if self.geometry == 'line' or self.geometry == 'line2':
            d_real = d#self.v_n*DT
            a_i = [s_pose[0],s_pose[1]]
            a_i = [self.ax[-1],self.ay[-1]]
            
        else:
            d_real = self.v_n*DT
            a_i = [s_pose[0],s_pose[1]]
            a_i = [self.ax[-1],self.ay[-1]]
        
    
        for i in range(len(waypoints)):
            print('-----------------------------------------------------------------------',int(DT/self.dt))
            for j in range(int(DT/self.dt)):
                t_f = t_i+(waypoints[i]/int(DT/self.dt))
                tmp_x = np.cos(t_f)*self.v_n*(DT/self.dt)+a_i[0]
                tmp_y = np.sin(t_f)*self.v_n*(DT/self.dt)+a_i[1]
                self.ax.append(int(tmp_x))
                self.ay.append(int(tmp_y))
                t_i = t_f
                a_i = [tmp_x,tmp_y]
        #for i in range(int(DT/self.dt)-removed_waypoints):#-removed_waypoints
        if len(self.ax)>10:
            # Remove first waypoints (fixed path dimensions->computational load)
            self.ax.pop(0)
            self.ay.pop(0)
        # Generate new path 
        path = cubicSpline.CubicSpline2D(self.ax, self.ay) 

        return path, d_real, self.ax, self.ay, t_i

    def move_agents(self, path, d, s_pose, dt, auvs_xy, auvs_theta, r_yaw, r_x=0,r_y=0,bool=False):
        
        # Update Leader Position
        if config.geometry == 'column2' or config.geometry == 'column':
            s_pose[2] = r_yaw
            s_pose[0] = s_pose[0] + self.v_n*np.cos(s_pose[2])*dt
            s_pose[1] = s_pose[1] + self.v_n*np.sin(s_pose[2])*dt
        elif config.geometry == 'line2' or config.geometry == 'line':
            angular_vel_leader = (r_yaw-s_pose[2])
            s_pose[2] = (s_pose[2] + self.ko*angular_vel_leader*dt)
            #s_pose[2] = r_yaw
            s_pose[0] = s_pose[0] + self.v_n*np.cos(s_pose[2])*dt
            s_pose[1] = s_pose[1] + self.v_n*np.sin(s_pose[2])*dt
        # Update the position and orientation of the follower robots
        F_coop, desired_position = self.potential_field(path, s_pose, auvs_xy, d)
        # Compute the heading according to the desired position
        e_theta = self.compute_orientations(desired_position,auvs_xy, auvs_theta)
        for i in range(self.n_agents):
            auvs_theta[i] = auvs_theta[i] + self.ko*e_theta[i]*dt
            tmp1 = self.saturateVel((self.v_n*np.cos(auvs_theta[i]) + F_coop[i,0]*dt)*dt,bool)
            tmp2 = self.saturateVel((self.v_n*np.sin(auvs_theta[i]) + F_coop[i,1]*dt)*dt,bool)          
            auvs_xy[i,0] = auvs_xy[i,0] + F_coop[i,0]*dt*dt
            auvs_xy[i,1] = auvs_xy[i,1] + F_coop[i,1]*dt*dt
        if bool == True:
            auvs_xy = desired_position
            for i in range(self.n_agents):
                auvs_theta[i] = atan2(auvs_xy[i,1],auvs_xy[i,0])

        return s_pose, auvs_xy, auvs_theta
