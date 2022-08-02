import os
import importlib.util
import numpy as np
import copy
from src.main_cassino import MIN_TARGET_VEL, OPTIMIZATION_ON
# IMPORT GLOBAL VARIABLES FOR SIMULATION
spec = importlib.util.spec_from_file_location("module.main_cassino", "/home/andrea/ros_simulation_ws/src/ipp_pkg/src/main_cassino.py")
main = importlib.util.module_from_spec(spec)
spec.loader.exec_module(main)
TIME_STEP = main.TIME_STEP
TIME_COUNTER = main.TIME_COUNTER
OPTIMIZATION_ON = main.OPTIMIZATION_ON
MIN_TARGET_VEL = main.MIN_TARGET_VEL
MAX_TARGET_VEL = main.MAX_TARGET_VEL
Pose = main.Pose
target_x_traj = main.target_x_traj
target_y_traj = main.target_y_traj
N = main
platform_x = main.platform_x
platform_y = main.platform_y

def saturateVel(linear_velocity):
    if -MIN_TARGET_VEL < linear_velocity < MIN_TARGET_VEL:
        linear_velocity = MIN_TARGET_VEL
    if linear_velocity >= MAX_TARGET_VEL or linear_velocity <= -MAX_TARGET_VEL:
        linear_velocity = MAX_TARGET_VEL
    return np.abs(linear_velocity)

class Robot:
    """
    Constructs an instantiate the AUV

    Parameters
    ----------
    name : (string)
        The name of the robot
    color : (string)
        The color of the robot
    max_linear_speed : (float)
        The maximum linear speed that the robot can go
    max_angular_speed : (float)
        The maximum angular speed that the robot can rotate about its vertical
        axis
    controller : (Controller)
        A configurable controller to finds the path and calculates command
        linear and angular velocities. 
    """

    def __init__(self, name, color, path_finder_controller_auv,path_finder_controller_target):
        self.name = name
        self.color = color
        self.auv_controller = path_finder_controller_auv
        self.target_controller = path_finder_controller_target
        self.pose = Pose(0,0,0)
        self.pose_start = Pose(0,0,0)
        self.pose_target =Pose(0,0,0)
        self.lin_vel_target = 0
        self.ang_vel_target = 0

    def set_start_target_poses(self, pose_start, pose_target):
        """
        Sets the start and target positions of the robot

        Parameters
        ----------
        pose_start : (Pose)
            Start postion of the robot (see the Pose class)
        pose_target : (Pose)
            Target postion of the robot (see the Pose class)
        """
        self.pose_start = copy.copy(pose_start)
        self.pose_target = pose_target
        self.pose = pose_start

    def move_target(self, dt, curr_goal):
        """
        Moves the target for one time step increment

        Parameters
        ----------
        dt : (float)
            time step
        """
        global count1
        target_x_traj.append(self.pose_target.x)
        target_y_traj.append(self.pose_target.y)

        linear_velocity, angular_velocity = \
            self.target_controller .calc_control_command(
                curr_goal[0] - self.pose_target.x,
                curr_goal[1] - self.pose_target.y,
                self.pose_target.theta, curr_goal[2])

        linear_velocity = saturateVel(linear_velocity)
        self.pose_target.theta = self.pose_target.theta + angular_velocity * dt
        self.pose_target.x = self.pose_target.x + linear_velocity * \
            np.cos(self.pose_target.theta) * dt
        self.pose_target.y = self.pose_target.y + linear_velocity * \
            np.sin(self.pose_target.theta) * dt
        self.ang_vel_target = angular_velocity
        self.lin_vel_target = linear_velocity

    def move(self, dt, heading_changes, count1):
        """
        Moves the platform for one time step increment
        Parameters
        ----------
        dt : (float)
            time step
        heading_changes : (float)
            requested heading change
        """
        global count2, prev_count, goal_theta, old_pose
        
        platform_x.append(self.pose.x)
        platform_y.append(self.pose.y)
        flag = False

        if count2 == N: 
            count2 = 0
        if count1 >= (N*TIME_COUNTER):
            if count1%TIME_COUNTER == 0: #metti condizione di aspettare
                count2 = count2+1
        
        if prev_count != count2 and OPTIMIZATION_ON == True:
            print("RECEVEID NEW HEADING:*******************************************************************************", count2)
            heading_change = heading_changes[count2-1]
            if heading_change == 0:
                flag = True
            if count2 > 0:
                goal_theta = heading_change + old_pose
            linear_velocity, angular_velocity = \
            self.auv_controller.calc_control_command(
                0,
                0,
                self.pose.theta, goal_theta)
        else:
        
            angular_velocity = 0
            old_pose = self.pose.theta
        # Update State 
        linear_velocity = 1
        self.pose.theta = (self.pose.theta + angular_velocity)
        self.pose.x = self.pose.x + linear_velocity * \
            np.cos(self.pose.theta) * dt 
        self.pose.y = self.pose.y + linear_velocity * \
            np.sin(self.pose.theta) * dt
        # If theta reached be ready for the new cmd
        if (self.pose.theta == goal_theta or flag == True) and OPTIMIZATION_ON==True:         
            prev_count = count2
