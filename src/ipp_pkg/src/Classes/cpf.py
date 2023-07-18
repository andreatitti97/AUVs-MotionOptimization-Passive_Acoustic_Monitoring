import numpy as np
import matplotlib.pyplot as plt
import time, os
from math import atan2, pi
import importlib.util
# Import Costum classes
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
spec = importlib.util.spec_from_file_location("module.planner", class_path+"/spline_planner.py")
planner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(planner)
cubicSpline = planner

class CooperativePathFollowing:

    def __init__(self, formation, n_agents, init_orientation, k_att, k_rep, d_rep,lin_vel,bool):

        self.n_agents = n_agents
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_rep = d_rep
        self.ko = config.GAIN_YAW_RATE
        self.v_max = config.AUV_MAX_VEL
        self.desired_vel = lin_vel
        self.ds = config.TIME_STEP*config.TIME_SCALER #curve sampling, if higher path less smooth (i think)
        self.DT = config.MEAS_UPDATE*config.N_AUV*2 # TODO
        self.ax = []
        self.ay = []
        self.bool = bool


    def initialize_path(self, formation):
        if config.geometry == 'column' or config.geometry == 'column2':
            initial_path = [0, formation[2], formation[1], formation[0], formation[0]+self.DT+2]
            leader_path = cubicSpline.CubicSpline2D([0, formation[2], formation[1], formation[0]], [0,0,0,0])
            [rx, ry, ryaw, rk, s]=config.calc_spline_course(leader_path,self.ds)
            path_index = len(ryaw)

            distance = leader_path.s[-1] # initialize distance of the reference frame

        elif config.geometry == 'line' or config.geometry == 'line2':
            initial_path = [0, self.DT+2]
            distance = 0# initialize distance of the reference frame
            path_index = 0
        for i in range(len(initial_path)):
            self.ax.append(initial_path[i])
            self.ay.append(0)
        path = cubicSpline.CubicSpline2D(self.ax, self.ay)

        return path, path_index, distance

    def saturateVel(self,vel,bool=False):
        if bool == False:
            if vel > self.v_max:
                print('SATURATED VELS +++++++++++++++++++++++++++++++++++++++++++++++++ ',vel)
                vel = self.v_max

            if vel < -self.v_max:
                print('SATURATED VELS ------------------------------------------------- ',vel)
                vel = -self.v_max         
        return vel

    def potential_field(self, path, leader_pos,pos, leader_distance):
        # Calculate the desired positions of the followers in the formation
        formation = config.formation
        desired_positions = np.zeros_like(pos)
        # Compute the desired absolute pos of the agents according to leader pos and given formation
        if config.geometry == 'line' or config.geometry == 'line2':
            for i in range(0, self.n_agents):
                desired_positions[i,0] = leader_pos[0] + config.a*(formation[i,0]*np.cos(leader_pos[2])+formation[i,1]*np.sin(leader_pos[2]))
                desired_positions[i,1] = leader_pos[1] + config.b*(-formation[i,0]*np.sin(leader_pos[2])+formation[i,1]*np.cos(leader_pos[2]))
        elif config.geometry == 'column' or config.geometry == 'column2':
            for i in range(len(formation)):
                x,y = path.calc_position(-formation[i]+leader_distance)

                desired_positions[i,0] = x
                desired_positions[i,1] = y

        # Calculate the attractive potential for each robot
        F_att = -self.k_att * (pos - desired_positions)

        # Calculate the repulsive potential for each robot
        F_rep = np.zeros_like(F_att)
        for i in range(self.n_agents):
            for j in range(i+1, self.n_agents):
                d = np.linalg.norm(pos[i] - pos[j])
                if d < self.d_rep:
                    F_rep[i] += self.k_rep * (1/(d+1e8) - 1/self.d_rep) * (pos[i] - pos[j]) / (d+1e8)
                    F_rep[j] += self.k_rep * (1/(d+1e8) - 1/self.d_rep) * (pos[j] - pos[i]) / (d+1e8)
        
        # Calculate the total force for each robot
        F_total = F_att + F_rep
           
        return F_total, desired_positions

    def compute_orientations(self, desired_pos, pos, num_robots, orientations):
        orientations_goal = np.zeros((self.n_agents))
        error_ang = np.zeros((self.n_agents))
        for i in range(self.n_agents):
            # Compute the direction vector from the follower's current position to its desired position
            orientations_goal[i] = atan2(pos[i,1]-desired_pos[i,1],pos[i,0]-desired_pos[i,0])
            error_ang[i] = (orientations_goal[i] - orientations[i])

        return error_ang

    def update_path(self, waypoints, init_theta):
        init_pose = [self.ax[-1],self.ay[-1]]

        for i in range(len(waypoints)):

            theta_goal = init_theta+waypoints[i]

            tmp_x = np.cos(theta_goal)*self.desired_vel*self.DT+init_pose[0]
            tmp_y = np.sin(theta_goal)*self.desired_vel*self.DT+init_pose[1]
            self.ax.append(tmp_x)
            self.ay.append(tmp_y)
            init_theta = theta_goal
            init_pose = [tmp_x, tmp_y]
            
        path = cubicSpline.CubicSpline2D(self.ax, self.ay)
        return path, self.ax, self. ay, init_theta

    def move_agents(self, path, distance, leader_pos, dt, positions, orientations, r_yaw, bool=False):
        

        #TODO: move agents inside optimization


        angular_vel_leader = (r_yaw-leader_pos[2])
        # Update Leader Position

        leader_pos[2] = (leader_pos[2] + self.ko*angular_vel_leader*dt)
        leader_pos[2] = r_yaw
        leader_pos[0] = leader_pos[0] + self.desired_vel*np.cos(leader_pos[2])*dt
        leader_pos[1] = leader_pos[1] + self.desired_vel*np.sin(leader_pos[2])*dt


        
        # Update the position and orientation of the follower robots
        F_total_follower, desired_position = self.potential_field(path, leader_pos, positions, distance)
        # Compute the heading according to the desired position
        error_angular = self.compute_orientations(desired_position,positions, self.n_agents, orientations)
        #print('F_TOTAL',F_total_follower)
        for i in range(self.n_agents):
            orientations[i] = orientations[i] + self.ko/2*error_angular[i]*dt
            tmp1 = self.saturateVel((self.desired_vel*np.cos(orientations[i]) + F_total_follower[i,0]*dt)*dt,bool)
            tmp2 = self.saturateVel((self.desired_vel*np.sin(orientations[i]) + F_total_follower[i,1]*dt)*dt,bool)          
            positions[i,0] = positions[i,0] + tmp1
            positions[i,1] = positions[i,1] + tmp2
#        print('AUVs POSITION',positions)
        return leader_pos, positions, orientations
