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


count2, prev_count = 0, 0
goal_theta, old_pose = 0, 0
angular_velocity = 0

class CooperativePathFollowing:

    def __init__(self, formation, n_agents, init_orientation, k_att, k_rep, d_rep,lin_vel):
        self.formation = formation
        self.n_agents = n_agents
        self.theta = init_orientation
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_rep = d_rep
        self.desired_vel = lin_vel

    def update_leader_ori(self,updated_ori):
        self.theta = updated_ori
        
    def potential_field(self,leader_pos, pos):
        # Calculate the desired positions of the followers in the formation
    
        formation = np.array([[config.PLATFORM_INIT_POSE[0], config.PLATFORM_INIT_POSE[1]],[config.BASELINE_X, -config.BASELINE_Y], 
                            [config.BASELINE_X, config.BASELINE_Y], 
                            [config.BASELINE_X, -config.BASELINE_Y*2], 
                            [config.BASELINE_X, config.BASELINE_Y*2]])

        desired_positions = np.zeros_like(pos)

        for i in range(0, self.n_agents):
            desired_positions[i] = leader_pos + formation[i+1]
        
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
        angular_vel = np.zeros((self.n_agents))
        for i in range(self.n_agents):
            # Compute the direction vector from the follower's current position to its desired position
            orientations_goal[i] = atan2(desired_pos[i,1]-pos[i,1],desired_pos[i,0]-pos[i,0])
            angular_vel[i] = orientations_goal[i] - orientations[i]

        return angular_vel

    def move_agents(self, leader_pos, ctrl_cmd, dt, positions, orientations, count1, bool=False):
        
        global count2, prev_count, goal_theta, old_pose, angular_velocity
        
        
        if bool == True:
            if (np.abs(np.round(goal_theta,3) - np.round(self.theta,3)) < 0.02) and count1 == 8:         
                count2 = 0
                angular_vel_leader = 0
            else:
                count2  = 1
        else: 
            if (np.abs(np.round(goal_theta,3) - np.round(self.theta,3)) < 0.02) and count1 > 15:         
                count2 = 0
                angular_vel_leader = 0            
            if count1%config.STATE_PROPAGATION == 0 and count1 >= config.STATE_PROPAGATION:
                count2 = 1    
        # Compute the angular velocity after received the command
        if prev_count != count2 and config.OPTIMIZATION_ON == True:

            ko = config.CONTROLLER_GAIN

            goal_theta = ctrl_cmd + old_pose
            angular_vel_leader = ko*(goal_theta - self.theta)
        else:
            angular_vel_leader = 0
            old_pose = self.theta

        # Update Leader Position
        self.theta = (self.theta + angular_vel_leader*dt)
        
        tmp1 = leader_pos[0] + self.desired_vel*np.cos(self.theta)*dt
        tmp2 = leader_pos[1] + self.desired_vel*np.sin(self.theta)*dt

        leader_pos = [tmp1, tmp2]

        # Update the position and orientation of the follower robots
        F_total_follower, desired_position = self.potential_field(leader_pos, positions)
        # Compute the heading according to the desired position
        angular_vel = self.compute_orientations(desired_position,positions, self.n_agents, orientations)
        for i in range(self.n_agents):
            orientations[i] = orientations[i] + angular_vel[i]*dt
            tmp1 = positions[i,0] + (self.desired_vel*np.cos(orientations[i]) + F_total_follower[i,0]*dt)*dt 
            tmp2 = positions[i,1] + (self.desired_vel*np.sin(orientations[i]) + F_total_follower[i,1]*dt)*dt 
            positions[i,0] = tmp1
            positions[i,1] = tmp2
        return leader_pos, self.theta, positions, orientations
