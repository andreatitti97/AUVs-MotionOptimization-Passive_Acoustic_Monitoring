#Import basic system modules
import os
import sys
# Import math modules
from re import T
import matplotlib.pyplot as plt
from math import pi
import numpy as np
import copy
#Import ROS modules
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
# Import Costum classes
from Classes.tracker import Tracker
from Classes.controller import Controller
from Classes.sensor import Sensor
# PATH DEFINITON
utils_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs/utils')
sys.path.append(utils_path)
plot_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs/plot')
sys.path.append(plot_path)

# Simulation parameters
TIME_DURATION = 61
TIME_STEP = 0.01
SHOW_ANIMATION = False
PLOT_WINDOW_SIZE_X = 900
PLOT_WINDOW_SIZE_Y = 900
PLOT_FONT_SIZE = 10
t = 0
simulation_running = True

#GLOBAL VARIABLES
x0 = [30, 100, pi/3]
x0_dot = [5, 0]
s0 = [800, 450, pi]
count = 0
prev_count = 0
goal_theta = 0
old_pose  = 0
N = 4 #planning horizon
target_x_traj = []
target_y_traj = []
rmse_x = []
rmse_y = []
target_est_x = []
target_est_y = []

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

    def __init__(self, name, color, max_linear_speed, max_angular_speed,
                 path_finder_controller):
        self.name = name
        self.color = color
        self.MAX_LINEAR_SPEED = max_linear_speed
        self.MAX_ANGULAR_SPEED = max_angular_speed
        self.path_finder_controller = path_finder_controller
        self.x_traj = []
        self.y_traj = []
        self.target_x_traj = []
        self.target_y_traj = []
        self.pose = Pose(0,0,0)
        self.pose_start = Pose(0,0,0)
        self.pose_target =Pose(0,0,0)

        self.vel_lin_target = x0_dot[0]
        self.vel_ang_target = 0
        self.is_at_target = False

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
        
        target_x_traj.append(self.pose_target.x)
        target_y_traj.append(self.pose_target.y)
        linear_velocity = self.vel_lin_target
        angular_velocity = self.vel_ang_target
        self.pose_target.theta = self.pose_target.theta + angular_velocity * dt
        self.pose_target.x = self.pose_target.x + linear_velocity * \
            np.cos(self.pose_target.theta) * dt
        self.pose_target.y = self.pose_target.y + linear_velocity * \
            np.sin(self.pose_target.theta) * dt

    def move(self, dt, heading_changes):
        """
        Moves the robot for one time step increment

        Parameters
        ----------
        dt : (float)
            time step
        """
        global t, count, prev_count, goal_theta, old_pose
        self.x_traj.append(self.pose.x)
        self.y_traj.append(self.pose.y)
        t += 1
        
        if t%200 == 0:
            count = count+1

        if count == N+1: 
            count = 1

        if prev_count != count:
        
            #print('provs')
            heading_change = heading_changes[count-1]
            if count > 1:
                goal_theta = heading_change + old_pose  #heading_changes[count-2]
            else: 
                goal_theta = heading_change

            #heading_change = heading_change+self.pose.theta 
            rho, linear_velocity, angular_velocity = \
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

        if np.abs(angular_velocity) < 0.15:
            prev_count = count
        


def run_simulation(robots, tracker1, sensor1, sensor2, pub_estimation, pub_platform_state, pub_covariance):
    """Simulate the sensor platform and the moving target"""
    
    global simulation_running
    rate = rospy.Rate(100) #loop spin at 100 Hz
    # Init Time Variables
    t = 0    
    while simulation_running is True and t <= TIME_DURATION:
        
        t += TIME_STEP
        for instance in robots:
        # SIMULATE SENSORS MEASURAMENTS
            sensor1.vehiclePose(instance.pose.x,instance.pose.y,instance.pose.theta)
            sensor1.targetPoseNoisy(instance.pose_target.x,instance.pose_target.y,instance.pose_target.theta)
            [measure1,sensor_pose] = sensor1.measureBearing()
            
            sensor2.vehiclePose(instance.pose.x,instance.pose.y,instance.pose.theta)
            sensor2.targetPoseNoisy(instance.pose_target.x,instance.pose_target.y,instance.pose_target.theta)
            [measure2, sensor_pose2] = sensor2.measureBearing()  
            
            measures = [measure1, measure2]
            target_state = [instance.pose_target.x,instance.pose_target.y, x0_dot[0]*np.cos(instance.pose_target.theta),
         x0_dot[0]*np.sin(instance.pose_target.theta)]

        # SIMULATE EKF
        tracker1.processMeasurement(measures,target_state, sensor_pose, sensor_pose2, TIME_STEP)
        [curr_est, P] = tracker1.state
        
        # PUBLISH INFORMATION FOR OPTIMIZATION
        cov = []
        pub_estimation.publish(np.array(curr_est,dtype=np.float32))
        for i in range(4):
            for j in range(4):
                cov.append(P[i,j])

        pub_covariance.publish(np.array(cov,dtype=np.float32))
        pub_platform_state.publish(np.array(sensor_pose,dtype=np.float32))
        # LOAD SEQUENCE OF CTRL_CMD FROM OPTIMIZATION
        ctrl_cmd = np.loadtxt(utils_path+'/ctrl_cmd.txt')
        print(ctrl_cmd)
        # SAVE DATA FOR PLOT
        err_y = np.sqrt(((target_state[1] - curr_est[1,0])**2))
        err_x = np.sqrt(((target_state[0] - curr_est[0,0])**2))

        target_est_y.append(curr_est[1,0])
        target_est_x.append(curr_est[0,0])
        rmse_x.append(err_y)
        rmse_y.append(err_x)
        
        
        instance.move(TIME_STEP, ctrl_cmd)
        instance.move_target(TIME_STEP)
        if t > 60:
            print('saving data for plot')
            np.savetxt(plot_path+'/target_x_traj.txt',target_x_traj)
            np.savetxt(plot_path+'/target_y_traj.txt',target_y_traj)
            np.savetxt(plot_path+'/target_est_x.txt',target_est_x)
            np.savetxt(plot_path+'/target_est_y.txt',target_est_y)
            np.savetxt(plot_path+'/rmse_y.txt',rmse_y)
            np.savetxt(plot_path+'/rmse_x.txt',rmse_x)
        rate.sleep()


        if SHOW_ANIMATION:
            plt.cla()
            plt.xlim(0, PLOT_WINDOW_SIZE_X)
            plt.ylim(0, PLOT_WINDOW_SIZE_Y)

            # For stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.text(0.3, PLOT_WINDOW_SIZE_Y - 1,
                     'Time: {:.2f}'.format(t),
                     fontsize=PLOT_FONT_SIZE)

            for instance in robots:
                plt.arrow(instance.pose_start.x,
                            instance.pose_start.y,
                            np.cos(instance.pose_start.theta),
                            np.sin(instance.pose_start.theta),
                            color='r',
                            width=1)
                plt.arrow(instance.pose.x,
                            instance.pose.y,
                            np.cos(instance.pose.theta),
                            np.sin(instance.pose.theta),
                            color='g',
                            width=1)

                plot_vehicle(sensor_pose[0],
                                sensor_pose[1],
                                sensor_pose[2],
                                instance.x_traj,
                                instance.y_traj,
                                color='r')

                plot_vehicle(sensor_pose2[0],
                                sensor_pose2[1],
                                sensor_pose2[2],
                                instance.x_traj,
                                instance.y_traj,
                                color='g')
                          

                plot_vehicle(instance.pose.x,
                                instance.pose.y,
                                instance.pose.theta,
                                instance.x_traj,
                                instance.y_traj, instance.color)

                plt.arrow(instance.pose_target.x,
                            instance.pose_target.y,
                            np.cos(instance.pose_target.theta),
                            np.sin(instance.pose_target.theta),
                            color='r',
                            width=1)
                plt.arrow(instance.pose_target.x,
                            instance.pose_target.y,
                            np.cos(instance.pose_target.theta),
                            np.sin(instance.pose_target.theta),
                            color='g',
                            width=1)
                
                plot_vehicle(instance.pose_target.x,
                                instance.pose_target.y,
                                instance.pose_target.theta,
                                instance.x_traj,
                                instance.y_traj, 
                                instance.color)
            
            plt.pause(TIME_STEP)
            

def plot_vehicle(x, y, theta, x_traj, y_traj, color):
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = wTv(x, y, theta)
    p1 = T @ p1_i
    p2 = T @ p2_i
    p3 = T @ p3_i

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color+'-',linewidth=3)
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], color+'-',linewidth=3)
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], color+'-',linewidth=3)

    plt.plot(x_traj, y_traj, color+'--', linewidth=1)


def wTv(x, y, theta):
    ''' Funzione che ritorna la trasformate
     tra il frame mondo e il veicolo
    '''
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

def callback(data):
    ctrl_cmd = data.data
    #print(ctrl_cmd)
    np.savetxt(utils_path+'/ctrl_cmd.txt',np.array(ctrl_cmd,dtype=np.float32))

def main():
    # ROS INIT
    rospy.init_node('simulation')
    pub_estimation = rospy.Publisher('estimation', numpy_msg(Floats), queue_size=100)
    pub_covariance = rospy.Publisher('covariance', numpy_msg(Floats), queue_size=1000)
    pub_platform_state = rospy.Publisher('platform_state', numpy_msg(Floats), queue_size=100)
    rospy.Subscriber('ctrl_cmd',numpy_msg(Floats), callback)
    
    # Initial Conditions
    pose_target = Pose(x0[0], x0[1], x0[2])
    pose_start_1 = Pose(s0[0], s0[1], s0[2])
    controller= Controller(5, 8, 2) # controller parameters 
    robot_1 = Robot("platoform_center", "y", 100, 100, controller)
   
    # Sensor Initialization
    
    f1 = 1 #Hz
    f2 = 1 #Hz
    mean1 = 0
    variance1 = 20
    mean2 = 0
    variance2 = 20
    sensor1 = Sensor('first_streamer',f1,mean1,variance1,1)#
    sensor2 = Sensor('seconda_streamer',f2,mean2,variance2,-1)
    tracker1 = Tracker('first_observer')
    
    # Set the AUV and the TARGET to the initial conditions
    robot_1.set_start_target_poses(pose_start_1, pose_target)
    # Instantiate the object Robot 
    robots: list[Robot] = [robot_1]
    # Run The Simulation
    
    run_simulation(robots, tracker1, sensor1, sensor2, pub_estimation, pub_platform_state, pub_covariance)

if __name__ == '__main__':
    main()
