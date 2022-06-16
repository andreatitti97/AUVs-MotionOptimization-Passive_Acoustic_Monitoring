#!/usr/bin/env python
#Import basic system modules
import os
import time
import importlib.util
# Import math modules
from re import T
from math import pi
import numpy as np
import copy
#Import ROS modules
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

# Import Costum classes
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.tracker", class_path+"/tracker.py")
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)
spec = importlib.util.spec_from_file_location("module.tracker", class_path+"/controller.py")
controller = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controller)
spec = importlib.util.spec_from_file_location("module.tracker", class_path+"/sensor.py")
sensor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor)
# PATH DEFINITON
utils_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/utils')
plot_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')

# Simulation parameters
TIME_DURATION = 1800 #seconds
TIME_STEP = 0.01
TIME_SCALER = 40 #max 20 for allow communication -> circa 9 minuti per simulare un ora 
TARGET_INIT = [8000, 8000, -pi/4, 3] #[x(m),y(m),theta(rad),linear vel(m/s)]
PLATFORM_INIT_POSE = [1000, 1000, 0] #[x,y,theta]
MEAS_VARIANCE = 0.1
OPTIMIZATION_ON = False
OPTIMIZATION_TIME_STEP = 8 
#GLOBAL VARIABLES
t = 0
N = 4 #planning horizon
# Internal counters
count2 = 0
count1 = 0
opt_counter = 0
prev_count = 0
goal_theta = 0
old_pose  = 0
# INIT lists for plot
target_x_traj = []
target_y_traj = []
platform_x = []
platform_y = []
rmse_x = []
rmse_y = []
rmse = []
target_est_x = []
target_est_y = []
auv1_x = []
auv1_y = []
auv2_x = []
auv2_y = []
bearing1 = []
bearing2 = []
bearing3 = []

class Pose:
    """2D pose"""

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

def wTv(x, y, theta):
    ''' Funzione che ritorna la trasformate
     tra il frame mondo e il veicolo
    '''
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

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

    def __init__(self, name, color, path_finder_controller):
        self.name = name
        self.color = color
        self.path_finder_controller = path_finder_controller
        self.pose = Pose(0,0,0)
        self.pose_start = Pose(0,0,0)
        self.pose_target =Pose(0,0,0)
        self.vel_lin_target = TARGET_INIT[3]
        self.vel_ang_target = 0

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

    def move_target(self, dt):
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

        # UNCOMMENT FOR CHANGE TARGET HEADING AFTER A WHILE
        #if count1 == 3025:#for change target heading after a while
         #   print('target heading change')
          #  self.pose_target =Pose(self.pose_target.x,self.pose_target.y,self.pose_target.theta + pi/4)
        
        linear_velocity = self.vel_lin_target
        angular_velocity = self.vel_ang_target
        self.pose_target.theta = self.pose_target.theta + angular_velocity * dt
        self.pose_target.x = self.pose_target.x + linear_velocity * \
            np.cos(self.pose_target.theta) * dt
        self.pose_target.y = self.pose_target.y + linear_velocity * \
            np.sin(self.pose_target.theta) * dt

    def move(self, dt, heading_changes):
        """
        Moves the platform for one time step increment

        Parameters
        ----------
        dt : (float)
            time step
        heading_changes : (float)
            requested heading change
        """
        global t, count2, prev_count, goal_theta, old_pose
        platform_x.append(self.pose.x)
        platform_y.append(self.pose.y)
        t += 1
        

        if count2 == N: 
            count2 = 0

        if t%(OPTIMIZATION_TIME_STEP*100/TIME_SCALER) == 0:
            count2 = count2+1
        

        if prev_count != count2:
            heading_change = heading_changes[count2-1]
            if count2 > 0:
                goal_theta = heading_change + old_pose
            else: 
                goal_theta = heading_change
            linear_velocity, angular_velocity = \
            self.path_finder_controller.calc_control_command(
                0,
                0,
                self.pose.theta, goal_theta)

        else:
        
            angular_velocity = 0
            old_pose = self.pose.theta
            

        linear_velocity = 1

        self.pose.theta = (self.pose.theta + angular_velocity * dt)
        
        self.pose.x = self.pose.x + linear_velocity * \
            np.cos(self.pose.theta) * dt 
  
        self.pose.y = self.pose.y + linear_velocity * \
            np.sin(self.pose.theta) * dt

        if 0.1 < np.abs(angular_velocity) < 0.2: #0.15 - 0.3
            prev_count = count2
        


def run_simulation(robots, tracker1, sensor1, sensor2, pub_estimation, pub_platform_state):
    """Simulate the sensor platform and the moving target"""
    global count1, opt_counter
    Hz = 1/(TIME_STEP) #NB: different from sampling rate for move things, this is ros rate
    rate = rospy.Rate(Hz)
    # Init Time Variables
    t = 0    
    count1 = 0
    while t <= TIME_DURATION:
        rospy.loginfo(t)
        t += TIME_STEP*TIME_SCALER
        count1 += 1
        for instance in robots:
        # SIMULATE SENSORS MEASURAMENTS
            sensor1.vehiclePose(instance.pose.x,instance.pose.y,instance.pose.theta, TIME_STEP)
            sensor1.targetPoseReal(instance.pose_target.x,instance.pose_target.y,instance.pose_target.theta)
            [measure1, vehicle_pose, rel_bearing1] = sensor1.measureBearing()
            
            sensor2.vehiclePose(instance.pose.x,instance.pose.y,instance.pose.theta, TIME_STEP)
            sensor2.targetPoseReal(instance.pose_target.x,instance.pose_target.y,instance.pose_target.theta)
            [measure2, vehicle_pose2, rel_bearing2] = sensor2.measureBearing()  
            measures = [measure1, measure2]
            target_state_real = [instance.pose_target.x,instance.pose_target.y, TARGET_INIT[3]*np.cos(instance.pose_target.theta),
          TARGET_INIT[3]*np.sin(instance.pose_target.theta)]

        # SIMULATE EKF
        if count1 == 1: #add distrubnace to th initial guess
            initial_gaussian_noise = np.random.normal(0, 10)
            initial_guess = [target_state_real[0] + initial_gaussian_noise, target_state_real[1] + initial_gaussian_noise,
                                target_state_real[2] + initial_gaussian_noise*0.01, target_state_real[3] + initial_gaussian_noise *0.01]
        tracker1.processMeasurement(measures,initial_guess, vehicle_pose, vehicle_pose2, TIME_STEP*TIME_SCALER)
        [curr_est, P] = tracker1.state
        print(curr_est)
        # PUBLISH INFORMATION FOR OPTIMIZATION
        # SEND LAST INFORMATIONS and LOAD SEQUENCE OF CTRL_CMD FROM OPTIMIZATION
        if count1 >= 100 and OPTIMIZATION_ON == True:
            if count1%(N*OPTIMIZATION_TIME_STEP/(TIME_STEP*TIME_SCALER)) == 0:
                
                rospy.loginfo('SENDING DATA')
                pub_estimation.publish(np.array(curr_est,dtype=np.float32))
                rospy.sleep(TIME_STEP*5)
                pub_platform_state.publish(np.array(vehicle_pose,dtype=np.float32))
                rospy.sleep(TIME_STEP*5)
                
                cmds = rospy.wait_for_message('ctrl_cmd',numpy_msg(Floats))
                cmds = cmds.data
                rospy.loginfo('RECEIVED CMDS')
                opt_counter += 1
            else: 
                cmds = [0, 0, 0, 0]
        else:
            cmds = [0, 0, 0, 0]
        # SAVE DATA FOR PLOT
        target_est_y.append(curr_est[1,0])
        target_est_x.append(curr_est[0,0])
        err_x = np.sqrt(((target_state_real[0] - curr_est[0,0])**2))
        err_y = np.sqrt(((target_state_real[1] - curr_est[1,0])**2))
        norma_err = np.sqrt(err_x**2+err_y**2)
        auv1_x.append(vehicle_pose[0])
        auv1_y.append(vehicle_pose[1])
        auv2_x.append(vehicle_pose2[0])
        auv2_y.append(vehicle_pose2[1])
        rmse.append(norma_err)
        bearing1.append(rel_bearing1)
        bearing2.append(rel_bearing2)
        instance.move(TIME_STEP*TIME_SCALER, cmds)
        instance.move_target(TIME_STEP*TIME_SCALER)
        if int(t) == (TIME_DURATION-1):
            rospy.loginfo('saving data for plot')
            np.savetxt(plot_path+'/target_x_traj.txt',target_x_traj)
            np.savetxt(plot_path+'/target_y_traj.txt',target_y_traj)

            if OPTIMIZATION_ON == True:
                np.savetxt(plot_path+'/target_est_x_ON.txt',target_est_x)
                np.savetxt(plot_path+'/target_est_y_ON.txt',target_est_y)
                np.savetxt(plot_path+'/rmse_ON.txt',rmse)
                np.savetxt(plot_path+'/x_platform_ON.txt',platform_x)
                np.savetxt(plot_path+'/y_platform_ON.txt',platform_y)
                np.savetxt(plot_path+'/auv1_x_ON.txt',auv1_x)
                np.savetxt(plot_path+'/auv1_y_ON.txt',auv1_y)
                np.savetxt(plot_path+'/auv2_x_ON.txt',auv2_x)
                np.savetxt(plot_path+'/auv2_y_ON.txt',auv2_y)
                np.savetxt(plot_path+'/bearing1_ON.txt',bearing1)
                np.savetxt(plot_path+'/bearing2_ON.txt',bearing2)
            else:
                np.savetxt(plot_path+'/target_est_x_OFF.txt',target_est_x)
                np.savetxt(plot_path+'/target_est_y_OFF.txt',target_est_y)
                np.savetxt(plot_path+'/rmse_OFF.txt',rmse)
                np.savetxt(plot_path+'/x_platform_OFF.txt',platform_x)
                np.savetxt(plot_path+'/y_platform_OFF.txt',platform_y)
                np.savetxt(plot_path+'/auv1_x_OFF.txt',auv1_x)
                np.savetxt(plot_path+'/auv1_y_OFF.txt',auv1_y)
                np.savetxt(plot_path+'/auv2_x_OFF.txt',auv2_x)
                np.savetxt(plot_path+'/auv2_y_OFF.txt',auv2_y)
                np.savetxt(plot_path+'/bearing1_OFF.txt',bearing1)
                np.savetxt(plot_path+'/bearing2_OFF.txt',bearing2)
        rate.sleep()

def main():
    # ROS INIT
    rospy.init_node('simulation')
    pub_estimation = rospy.Publisher('estimation', numpy_msg(Floats), queue_size=10)
    pub_platform_state = rospy.Publisher('platform_state', numpy_msg(Floats), queue_size=10)
    
    # Initial Conditions
    pose_target = Pose(TARGET_INIT[0], TARGET_INIT[1],  TARGET_INIT[2])
    pose_start_1 = Pose(PLATFORM_INIT_POSE[0], PLATFORM_INIT_POSE[1], PLATFORM_INIT_POSE[2])
    
    # Init tracker controller and robots
    tracker1 = tracker.Tracker('first_observer',False)
    controller1 = controller.Controller(5, 1) # controller parameters  (rho,alpha -> gain linear and angul vel) DO NOT CHANGE
    robot_1 = Robot("platoform_center", "y", controller1)
    
    # Sensor Initialization
    sensor1 = sensor.Sensor('first_streamer',1,0,MEAS_VARIANCE,1)#freq,mean,variance,displachement
    sensor2 = sensor.Sensor('seconda_streamer',1,0,MEAS_VARIANCE,-1)
    
    # Set the AUV and the TARGET to the initial conditions
    robot_1.set_start_target_poses(pose_start_1, pose_target)
    # Instantiate the object Robot 
    robots: list[Robot] = [robot_1]
    # Run The Simulation
    if OPTIMIZATION_ON == True:
        rospy.loginfo('LAUNCH THE OPTIMIZATION')
        time.sleep(3)# wait for optimization to launch
        rospy.loginfo('STARTED SIMULATION - OPTIMIZATION ON')
    else:
        rospy.loginfo('STARTED SIMULATION - OPTIMIZATION OFF')
    start = time.time()
    run_simulation(robots, tracker1, sensor1, sensor2, pub_estimation, pub_platform_state)
    stop = time.time()
    if OPTIMIZATION_ON == True:
        print('TOTAL SIMULATION TIME:',stop - start)
        print('AVG OPTIMIZATION TIME:',((stop - start)-TIME_DURATION)/opt_counter)
if __name__ == '__main__':
    main()
