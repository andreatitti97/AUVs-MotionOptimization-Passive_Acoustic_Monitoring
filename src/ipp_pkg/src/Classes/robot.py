import os 
import importlib
import numpy as np
import copy
import time
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)

count2, prev_count = 0, 0
goal_theta, old_pose = 0, 0
angular_velocity = 0
def saturateVel(linear_velocity):
    if config.MIN_TARGET_VEL < linear_velocity < config.MIN_TARGET_VEL:
        linear_velocity = config.MIN_TARGET_VEL
    if linear_velocity >= config.MAX_TARGET_VEL or linear_velocity <= -config.MAX_TARGET_VEL:
        linear_velocity = config.MAX_TARGET_VEL
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
        self.pose = config.Pose(0,0,0)
        self.pose_start = config.Pose(0,0,0)
        self.pose_target = config.Pose(0,0,0)
        self.lin_vel = 2
        self.ang_vel = 0
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
        #global count1
    
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
        global count2, prev_count, goal_theta, old_pose, angular_velocity
        self.lin_vel = 2
        if count2 == 1: 
            count2 = 0

        if count1%config.TIME_COUNTER == 0 and count1 >= config.TIME_COUNTER: #and count1 >= config.TIME_COUNTER
            count2 = count2+1
        
        if prev_count != count2 and config.OPTIMIZATION_ON == True:
            print("RECEVEID NEW HEADING:*******************************************************************************")
            heading_change = heading_changes[0] #apply only the first command (MPC paradigm)

            goal_theta = heading_change + old_pose
            linear_velocity, angular_velocity = \
            self.auv_controller.calc_control_command(
                0,
                0,
                self.pose.theta, goal_theta)
            self.ang_vel = angular_velocity
            #self.lin_vel = linear_velocity
        else:
            
            old_pose = self.pose.theta
        # Update State 

        self.pose.theta = (self.pose.theta + self.ang_vel)
        self.pose.x = self.pose.x + self.lin_vel * \
            np.cos(self.pose.theta) * dt 
        self.pose.y = self.pose.y + self.lin_vel * \
            np.sin(self.pose.theta) * dt
        
        '''if self.ang_vel > 0: #DEBUGGING PURPOSE
            print('ang vel', self.ang_vel)
            print('old pose',old_pose)
            print('pose',self.pose.theta)
            print('goal_theta',goal_theta)
            print('difference',np.abs(np.round(goal_theta,2) - np.round(self.pose.theta,2)))'''

        # If theta reached be ready for the new cmd
        if (np.abs(np.round(goal_theta,3) - np.round(self.pose.theta,3)) < 0.05) and config.OPTIMIZATION_ON==True:         
            prev_count = count2
            self.ang_vel = 0
            print('REACHED GOAL ------------------------------------------------------------------------------------------------')
            