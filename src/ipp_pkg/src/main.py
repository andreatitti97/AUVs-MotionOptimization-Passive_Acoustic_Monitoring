#!/usr/bin/env python
#Import basic system modules
import os
import time
import importlib.util
# Import math modules
from math import pi
import numpy as np
import copy

#Import ROS modules
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

# Import Costum classes
class_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.tracker", class_path+"/TRACKER_distributed.py")
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)
spec = importlib.util.spec_from_file_location("module.controller", class_path+"/controller.py")
controller = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controller)
spec = importlib.util.spec_from_file_location("module.sensor", class_path+"/sensor.py")
sensor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor)
# PATH DEFINITON
utils_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/utils')
plot_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/ipp_pkg/src/logs/plot')

# Simulation parameters
TIME_DURATION = 2900 #seconds#2600
TIME_STEP = 0.01
TIME_SCALER = 80 # MAX for communication purpose 
TARGET_INIT = [-4000, -3500, 0, 5] #[x(m),y(m),theta(rad),linear vel(m/s)]
PLATFORM_INIT_POSE = [1000, 1000, 0] #[x,y,theta]
MEAS_VARIANCE = 0.01 #already al quadrato -> 2° incertezza -> sigma^2 = (2*pi/180)^2
OPTIMIZATION_ON = True
OPTIMIZATION_TIME_STEP = 128 #VA INTESO COME time between each command 
BASELINE_Y = 1200
BASELINE_X = 200
INIT_POSE_UNCERTAINTY = 50 #(m)
INIT_VEL_UNCERTAINTY = 0.01 #(m/s)
EKF_MEAS_UPDATE = 10 #(s) delta time tra le misure
N_AUV = 4
MAX_TARGET_VEL = 8 #(m/s)
MIN_TARGET_VEL = 3 #(m/s)
#GLOBAL VARIABLES
t = 0
N = 4 #planning horizon
# Internal counters
count2, count1, prev_count  = 0, 0, 0
goal_theta, old_pose = 0, 0
# INIT lists for plot
target_x_traj, target_y_traj, platform_x, platform_y = [], [], [], []
target_est_x, target_est_y,target_est_x2, target_est_y2, rmse = [], [], [], [], []
auv1_x, auv1_y, auv2_x, auv2_y,auv3_x,auv3_y,auv4_x,auv4_y  = [], [], [], [], [], [], [], []
bearing1, bearing2 = [], []


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


def sensorPlacement(auv):
    for i in range(N_AUV): #TODO: AUV up to 6 consider
            if (i+1) % 2 == 0:
                if i+1 > 3:
                    auv.append(sensor.Sensor(str(i),1,0,MEAS_VARIANCE,-1,BASELINE_X, BASELINE_Y))#freq,mean,variance,displachement
                else:   
                    auv.append(sensor.Sensor(str(i),1,0,MEAS_VARIANCE,-1,0,BASELINE_Y))#freq,mean,variance,displachement
            if (i+1) % 2 == 1:
                if i+1 > 2:
                    auv.append(sensor.Sensor(str(i),1,0,MEAS_VARIANCE,1,BASELINE_X, BASELINE_Y))#freq,mean,variance,displachement
                else:   
                    auv.append(sensor.Sensor(str(i),1,0,MEAS_VARIANCE,1,0,BASELINE_Y))#freq,mean,variance,displachement
    return auv

def saturateVel(linear_velocity):
    if -MIN_TARGET_VEL < linear_velocity < MIN_TARGET_VEL:
        linear_velocity = MIN_TARGET_VEL
    if linear_velocity >= MAX_TARGET_VEL or linear_velocity <= -MAX_TARGET_VEL:
        linear_velocity = MAX_TARGET_VEL
    return np.abs(linear_velocity)

class Pose:
    """2D pose"""

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

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
        self.waypoints = []

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
        if count1 >= (N*OPTIMIZATION_TIME_STEP/(TIME_STEP*TIME_SCALER)):
            if count1%(OPTIMIZATION_TIME_STEP/(TIME_STEP*TIME_SCALER)) == 0: #metti condizione di aspettare
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

def run_simulation(robots, obs, auv, pub_estimation, pub_platform_state, pub_covariance, poly_traj):
    """Simulate the sensor platform and the moving target"""
    global count1
    Hz = 1/(TIME_STEP) #NB: different from sampling rate for move things, this is ros rate
    rate = rospy.Rate(Hz)
    # Init Time Variables
    t = 0    
    count1 = 0
    cmds = []
    while t <= TIME_DURATION:
        rospy.loginfo('SIMULATION TIME(s)')
        rospy.loginfo(t)
        t += TIME_STEP*TIME_SCALER
        count1 += 1
        measures = []
        vehicle_pose = []
        rel_bearing = []
        for instance in robots:
        # SIMULATE SENSORS MEASURAMENTS
            for i in range(len(auv)):
                auv[i].vehiclePose(instance.pose.x,instance.pose.y,instance.pose.theta)
                auv[i].targetPoseReal(instance.pose_target.x,instance.pose_target.y,instance.pose_target.theta)
                [measure1, vehicle_pose1, rel_bearing1] = auv[i].measureBearing()
                measures.append(measure1)
                vehicle_pose.append(vehicle_pose1)
                rel_bearing.append(rel_bearing1)
    
            platform_pose = np.array([instance.pose.x,instance.pose.y,instance.pose.theta])
            target_state_real = [instance.pose_target.x,instance.pose_target.y, TARGET_INIT[3]*np.cos(instance.pose_target.theta),
          TARGET_INIT[3]*np.sin(instance.pose_target.theta)]

        # SIMULATE EKF
        if count1 == 1: #add distrubnace to th initial guess GAUSSIAN DISTURB TO INITIAL STATE
            initial_gaussian_noise = np.random.normal(0,INIT_POSE_UNCERTAINTY) #DO NOT CHANGE (m) - ekf tunato con questi valori, se da alzare cambiare EKF
            initial_gaussian_noise_vel = np.random.normal(0,INIT_VEL_UNCERTAINTY) #DO NOT CHANGE (m/s)
            initial_guess = [target_state_real[0] + initial_gaussian_noise, target_state_real[1] + initial_gaussian_noise,
                                target_state_real[2] + initial_gaussian_noise_vel, target_state_real[3] + initial_gaussian_noise_vel]#target_state_real[3] + initial_gaussian_noise *0.01
        
        if count1 == 1:
            for i in range(len(auv)):
                obs[i].processMeasurement(measures,initial_guess, vehicle_pose, TIME_SCALER*TIME_STEP, True) #FIRST UPDATE

        if count1 % 10:
            for i in range(len(auv)):
                obs[i].processMeasurement(measures[i],initial_guess, vehicle_pose[i], 2*TIME_SCALER*TIME_STEP, False)#LOCAL UPDATE
                
        if count1 % 120: #TODO update EKF not always
            for i in range(len(auv)):
                obs[i].processMeasurement(measures,initial_guess, vehicle_pose, 2*EKF_MEAS_UPDATE*TIME_SCALER*TIME_STEP, True)#DISTRIBUTED UPDATE
            
         #TODO considera covarianze di tutti e stato di tutti pre ottimizzazione
        [curr_est1, P1] = obs[0].state
        [curr_est2, P2] = obs[1].state
        if N_AUV > 2:
            [curr_est3, P3] = obs[2].state
            [curr_est4, P4] = obs[3].state
        
        # PUBLISH INFORMATION FOR OPTIMIZATION
        # SEND LAST INFORMATIONS and LOAD SEQUENCE OF CTRL_CMD FROM OPTIMIZATION

        if count1%((N*OPTIMIZATION_TIME_STEP)/(TIME_STEP*TIME_SCALER)) == 0 and OPTIMIZATION_ON == True: #multiplo di 640 con OPT_dt = 128

            cov_values = np.array([P1[0,0],P1[1,1],P1[2,2],P1[3,3]])
            rospy.loginfo('SENDING DATA')
            pub_estimation.publish(np.array(curr_est1,dtype=np.float32))
            rospy.sleep(TIME_STEP*5)
            pub_platform_state.publish(np.array(platform_pose,dtype=np.float32))
            rospy.sleep(TIME_STEP*5)
            pub_covariance.publish(np.array(cov_values, dtype=np.float32))
            cmds = rospy.wait_for_message('ctrl_cmd',numpy_msg(Floats))
            cmds = cmds.data
            rospy.loginfo('RECEIVED CMDS')
            print(cmds)

        # SAVE DATA FOR PLOT
        target_est_y.append(curr_est1[1,0])
        target_est_x.append(curr_est1[0,0])
        
        err_x = np.sqrt(((target_state_real[0] - curr_est1[0,0])**2))
        err_y = np.sqrt(((target_state_real[1] - curr_est1[1,0])**2))
        norma_err = np.sqrt(err_x**2+err_y**2)
        for i in range(len(vehicle_pose)):  
            tmp = vehicle_pose[i]
            if i == 0:
                auv1_x.append(tmp[0])
                auv1_y.append(tmp[1])
            if i == 1:
                auv2_x.append(tmp[0])
                auv2_y.append(tmp[1])
            if i == 2:
                auv3_x.append(tmp[0])
                auv3_y.append(tmp[1])
            if i == 3:
                auv4_x.append(tmp[0])
                auv4_y.append(tmp[1])

        rmse.append(norma_err)
        bearing1.append(rel_bearing1)
        ######################################### MOVE THE ROBOTS #######################################################
        instance.move(TIME_STEP*TIME_SCALER, cmds, count1)
        instance.move_target(TIME_STEP*TIME_SCALER,poly_traj[count1])
        #################################################################################################################
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
                np.savetxt(plot_path+'/auv3_x_ON.txt',auv3_x)
                np.savetxt(plot_path+'/auv3_y_ON.txt',auv3_y)
                np.savetxt(plot_path+'/auv4_x_ON.txt',auv4_x)
                np.savetxt(plot_path+'/auv4_y_ON.txt',auv4_y)
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
                np.savetxt(plot_path+'/auv3_x_OFF.txt',auv3_x)
                np.savetxt(plot_path+'/auv3_y_OFF.txt',auv3_y)
                np.savetxt(plot_path+'/auv4_x_OFF.txt',auv4_x)
                np.savetxt(plot_path+'/auv4_y_OFF.txt',auv4_y)
                np.savetxt(plot_path+'/bearing1_OFF.txt',bearing1)
                np.savetxt(plot_path+'/bearing2_OFF.txt',bearing2)
        rate.sleep()

def main():
    # ROS INIT
    rospy.init_node('simulation')
    pub_estimation = rospy.Publisher('estimation', numpy_msg(Floats), queue_size=10)
    pub_platform_state = rospy.Publisher('platform_state', numpy_msg(Floats), queue_size=100)
    pub_covariance = rospy.Publisher('covariance_values', numpy_msg(Floats), queue_size=100)
    # Initial Conditions
    pose_target = Pose(TARGET_INIT[0], TARGET_INIT[1],  TARGET_INIT[2])
    pose_start_1 = Pose(PLATFORM_INIT_POSE[0], PLATFORM_INIT_POSE[1], PLATFORM_INIT_POSE[2])
    target_start = np.array([TARGET_INIT[0],TARGET_INIT[1], TARGET_INIT[2]])
    target_goal = np.array([4500, 3500,TARGET_INIT[2]+pi/10])
    # Init tracker controller and robots
    tr = []
    tracker1 = tracker.Tracker('first_observer',False, N_AUV)
    tracker2 = tracker.Tracker('second_observer',False, N_AUV)
    tr.append(tracker1)
    tr.append(tracker2)
    if N_AUV > 2:
        tracker3 = tracker.Tracker('third_observer',False, N_AUV)
        tracker4 = tracker.Tracker('forth_observer',False, N_AUV)
        tr.append(tracker3)
        tr.append(tracker4)
    controller1_target = controller.Controller(0.01, 0.1) # controller parameters  (rho,alpha -> gain linear and angul vel) DO NOT CHANGE
    controller1_auv = controller.Controller(1, 1)
    robot_1 = Robot("platoform_center", "y", controller1_auv, controller1_target)
    auv = []
    # Sensor Initialization
    auv = sensorPlacement(auv)
    # Set the AUV and the TARGET to the initial conditions
    robot_1.set_start_target_poses(pose_start_1, pose_target)
    # Instantiate the object Robot 
    robots: list[Robot] = [robot_1]
    # Generate Trajectory given the target start pos and waypoints
    ts = np.linspace(0,TIME_DURATION+1000,round(TIME_DURATION+1000/(TIME_SCALER*TIME_STEP)))

    #poly_traj = traj_generator.Trajectory(ts, target_start)
    [ts, poly_traj, vel, acc] = generatePolynomialTrajectory(ts, target_start, 0, 0, target_goal, 0, 0)

    # Run The Simulation
    if OPTIMIZATION_ON == True:
        rospy.loginfo('LAUNCH THE OPTIMIZATION')
        time.sleep(3)# wait for optimization to launch
        rospy.loginfo('STARTED SIMULATION - OPTIMIZATION ON')
    else:
        rospy.loginfo('STARTED SIMULATION - OPTIMIZATION OFF')

    run_simulation(robots, tr, auv, pub_estimation, pub_platform_state, pub_covariance, poly_traj)

if __name__ == '__main__':
    main()
