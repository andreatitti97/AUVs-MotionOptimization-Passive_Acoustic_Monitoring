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

class CooperativePathFollowing:

    def __init__(self, formation, n_agents, init_orientation, k_att, k_rep, d_rep,lin_vel):
        self.formation = formation
        self.n_agents = n_agents
        self.init_orientation = init_orientation
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_rep = d_rep
        self.desired_vel = lin_vel

    def potential_field(self,leader_pos, pos, num_robots, K_att, K_rep, d_rep):
        # Calculate the desired positions of the followers in the formation
    
        formation = np.array([[0, 0],[config.BASELINE_X, -config.BASELINE_Y], 
                            [config.BASELINE_X, config.BASELINE_Y], 
                            [config.BASELINE_X, -config.BASELINE_Y*2], 
                            [config.BASELINE_X, config.BASELINE_Y*2]])

        desired_positions = np.zeros_like(pos)

        for i in range(0, num_robots):
            desired_positions[i] = leader_pos + formation[i+1]
        
        # Calculate the attractive potential for each robot
        F_att = -K_att * (pos - desired_positions)
        
        # Calculate the repulsive potential for each robot
        F_rep = np.zeros_like(F_att)
        for i in range(self.n_agents):
            for j in range(i+1, self.n_agents):
                d = np.linalg.norm(pos[i] - pos[j])
                if d < d_rep:
                    F_rep[i] += K_rep * (1/(d+1e8) - 1/d_rep) * (pos[i] - pos[j]) / (d+1e8)
                    F_rep[j] += K_rep * (1/(d+1e8) - 1/d_rep) * (pos[j] - pos[i]) / (d+1e8)
        
        # Calculate the total force for each robot
        F_total = F_att + F_rep

        return F_total, desired_positions

    def compute_orientations(self, desired_pos, pos, num_robots, orientations):
        orientations_goal = np.zeros((self.n_agents))
        angular_vel = np.zeros((self.n_agents))
        for i in range(self.n_agents):
            # Compute the direction vector from the follower's current position to its desired position
            orientations_goal[i] = atan2(desired_pos[i,1]-pos[i,1],desired_pos[i,0]-pos[i,0])
            angular_vel[i] = orientations_goal[i] - orientations[i]

        return angular_vel

    def move_agents(self, leader_pos, leader_ori, ctrl_cmd, dt, positions, orientations):
        
        '''if ctrl_cmd != []:
            ko = config.CONTROLLER_GAIN
            theta_goal = ctrl_cmd
            angular_vel_leader = ko*(theta_goal - orientations[0])
        else:
            angular_vel_leader = 0'''
        angular_vel_leader = 0
        theta_leader = leader_ori + angular_vel_leader*dt

        
        tmp1 = leader_pos[0] + self.desired_vel*np.cos(theta_leader)*dt
        tmp2 = leader_pos[1] + self.desired_vel*np.sin(theta_leader)*dt
 
        leader_pos = [tmp1, tmp2]

        # Update the position and orientation of the follower robots
        F_total_follower, desired_position = self.potential_field(leader_pos, positions, self.n_agents, self.k_att, self.k_rep, self.d_rep)
        # Compute the heading according to the desired position
        angular_vel = self.compute_orientations(desired_position,positions, self.n_agents, orientations)
        for i in range(self.n_agents):
            orientations[i] = orientations[i] + angular_vel[i]*dt
            tmp1 = positions[i,0] + (self.desired_vel*np.cos(orientations[i]) + F_total_follower[i,0]*dt)*dt 
            tmp2 = positions[i,1] + (self.desired_vel*np.sin(orientations[i]) + F_total_follower[i,1]*dt)*dt 
            positions[i,0] = tmp1
            positions[i,1] = tmp2
        
        print('POSITIONS',positions)
        #time.sleep(5)
        return leader_pos, positions, orientations
