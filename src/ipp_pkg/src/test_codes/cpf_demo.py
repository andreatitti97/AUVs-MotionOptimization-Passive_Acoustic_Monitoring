import numpy as np
import matplotlib.pyplot as plt
import time, os
from math import atan2, pi


def generatePolynomialTrajectory(ts, y_from, yd_from, ydd_from, y_to, yd_to, ydd_to):
        
        a0 = y_from
        a1 = yd_from
        a2 = ydd_from / 2

        a3 = -10 * y_from - 6 * yd_from - 2.5 * ydd_from + 10 * y_to - 4 * yd_to + 0.5 * ydd_to
        a4 = 15 * y_from + 8 * yd_from + 2 * ydd_from - 15  * y_to  + 7 * yd_to - ydd_to
        a5 = -6 * y_from - 3 * yd_from - 0.5 * ydd_from  + 6 * y_to  - 3 * yd_to + 0.5 * ydd_to

        n_time_steps = ts.size
        n_dims = y_from.size
  
        ys = np.zeros([n_time_steps,n_dims])
        yds = np.zeros([n_time_steps,n_dims])
        ydds = np.zeros([n_time_steps,n_dims])

        for i in range(n_time_steps):
            t = (ts[i] - ts[0]) / (ts[n_time_steps - 1] - ts[0])
            ys[i,:] = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5)
            yds[i,:] = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4)
            ydds[i,:] = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3)

        yds /= (ts[n_time_steps - 1] - ts[0])
        ydds /= pow(ts[n_time_steps - 1] - ts[0], 2)

        return ts, ys, yds, ydds

def potential_field(leader_pos, pos, formation, num_robots, K_att, K_rep, d_rep):
    # Calculate the desired positions of the followers in the formation
 
    formation = np.array([[0, 0],[0, 250.0], [0, -250.0], [0, 500.0], [0, -500.0]])
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

def compute_orientations(desired_pos, pos, num_robots, orientations):
    orientations_goal = np.zeros((num_robots-1))
    angular_vel = np.zeros((num_robots-1))
    for i in range(num_robots-1):
        # Compute the direction vector from the follower's current position to its desired position
        orientations_goal[i] = atan2(desired_pos[i,1]-pos[i,1],desired_pos[i,0]-pos[i,0])
        angular_vel[i] = orientations_goal[i] - orientations[i]

    return angular_vel

def main():
    # Define the path for the leader robot to follow
    path_length = 10
    ts = np.linspace(0,path_length,10)   
    [ts, poly_traj, yd, ydd] = generatePolynomialTrajectory(ts,np.array([0,0,0]),0,0,np.array([10,0,0]),0,0)
    poly_traj_x = poly_traj[:,0]
    poly_traj_y = poly_traj[:,1]

    # Define the number of robots in the group
    num_robots = 5

    # Define the desired formation (equilateral triangle)
    formation = np.array([[0, 0],[0, 250.0], [0, -250.0], [0, 500.0], [0, -500.0]])

    # Define the initial position and orientation of the robots
    theta = 0
    positions = formation[1:num_robots]
    leader_pos = np.array([formation[0,0],formation[0,1]])
    orientations = np.zeros(num_robots)
    for i in range(num_robots):
        orientations[i] = theta

    # Define the gains for the control law
    K_att = 1.0 # Attractive gain
    K_rep = 50.0 # Repulsive gain
    d_rep = 0.2 # Distance threshold for repulsion

    pos1_x = []
    pos1_y = []
    pos2_x = []
    pos2_y = []
    pos3_x = []
    pos3_y = []
    pos4_x = []
    pos4_y = []
    pos5_x = []
    pos5_y = []
    
    v_lin = 1
    # Define the simulation loop
    # Define the simulation time and time step
    t_end = 100
    dt = 0.01
    t = 0
    count = 0
    angular_vel_leader = 0
    theta_goal = 0
    ko = 1.0
    while t < t_end:
        if count % 100 == 0:
            theta_goal += 0
            angular_vel_leader = ko*(theta_goal - orientations[0])
        # Update the position and orientation of the leader robot
        
        theta_leader = orientations[0] + angular_vel_leader*dt
        leader_pos[0] = leader_pos[0] + v_lin*np.cos(theta_leader)*dt
        leader_pos[1] = leader_pos[1] + v_lin*np.sin(theta_leader)*dt

        # Update the position and orientation of the follower robots
        F_total_follower, desired_position = potential_field(leader_pos, positions, formation, num_robots, K_att, K_rep, d_rep)
        # Compute the heading according to the desired position
        angular_vel = compute_orientations(desired_position,positions, num_robots, orientations[1:num_robots])
        for i in range(num_robots-1):
            orientations[i] = angular_vel[i]*dt
            tmp1 = positions[i,0] + (v_lin*np.cos(orientations[i]) + F_total_follower[i,0]*dt)*dt 
            tmp2 = positions[i,1] + (v_lin*np.sin(orientations[i]) + F_total_follower[i,1]*dt)*dt 
            positions[i,0] = tmp1
            positions[i,1] = tmp2
        # Update the simulation time and data structures for plot
        pos1_x.append(leader_pos[0])
        pos1_y.append(leader_pos[1])
        tmp1 = positions[0]
        #print(positions)
        #time.sleep(500)
        tmp2 = positions[1]
        tmp3 = positions[2]
        tmp4 = positions[3]
        pos2_x.append(tmp1[0])
        pos2_y.append(tmp1[1])
        pos3_x.append(tmp2[0])
        pos3_y.append(tmp2[1])
        pos4_x.append(tmp3[0])
        pos4_y.append(tmp3[1])
        pos5_x.append(tmp4[0])
        pos5_y.append(tmp4[1])

        t += dt
        count +=1
        print(count)
    plt.figure()
    plt.plot(pos1_x,pos1_y)
    plt.plot(pos2_x,pos2_y)
    plt.plot(pos3_x,pos3_y)
    plt.plot(pos4_x,pos4_y)
    plt.plot(pos5_x,pos5_y)
    plt.scatter(pos1_x[0],pos1_y[0])
    plt.scatter(pos2_x[0],pos2_y[0])
    plt.scatter(pos3_x[0],pos3_y[0])
    plt.scatter(pos1_x[-1],pos1_y[-1])
    plt.scatter(pos2_x[-1],pos2_y[-1])
    plt.scatter(pos3_x[-1],pos3_y[-1])
    plt.grid()
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    main()