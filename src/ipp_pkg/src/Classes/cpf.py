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
 
    formation = self.formation
    desired_positions = np.zeros_like(pos)

    for i in range(0, num_robots-1):
        desired_positions[i] = leader_pos + formation[i+1]
    
    # Calculate the attractive potential for each robot
    F_att = -K_att * (pos - desired_positions)
    
    # Calculate the repulsive potential for each robot
    F_rep = np.zeros_like(F_att)
    for i in range(num_robots-1):
        for j in range(i+1, num_robots-1):
            d = np.linalg.norm(pos[i] - pos[j])
            if d < d_rep:
                F_rep[i] += K_rep * (1/d - 1/d_rep) * (pos[i] - pos[j]) / d
                F_rep[j] += K_rep * (1/d - 1/d_rep) * (pos[j] - pos[i]) / d
    
    # Calculate the total force for each robot
    F_total = F_att + F_rep

    return F_total, desired_positions

def compute_orientations(self, desired_pos, pos, num_robots, orientations):
    orientations_goal = np.zeros((num_robots-1))
    angular_vel = np.zeros((num_robots-1))
    for i in range(num_robots-1):
        # Compute the direction vector from the follower's current position to its desired position
        orientations_goal[i] = atan2(desired_pos[i,1]-pos[i,1],desired_pos[i,0]-pos[i,0])
        angular_vel[i] = orientations_goal[i] - orientations[i]

    return angular_vel